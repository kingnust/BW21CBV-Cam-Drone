from __future__ import annotations

import json
import http.client
import socket
import threading
import time
import urllib.request
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterator, Protocol

import cv2
import numpy as np


@dataclass(frozen=True)
class EncodedFrame:
    sequence: int
    received_ns: int
    jpeg: bytes


@dataclass(frozen=True)
class FramePacket:
    sequence: int
    received_ns: int
    image: np.ndarray

    @property
    def age_ms(self) -> float:
        return (time.monotonic_ns() - self.received_ns) / 1_000_000.0


@dataclass
class SourceStats:
    connections: int = 0
    reconnects: int = 0
    bytes_received: int = 0
    frames_received: int = 0
    frame_batches: int = 0
    burst_batches: int = 0
    max_batch_frames: int = 0
    malformed_frames: int = 0
    oversized_buffers: int = 0
    last_error: str = ""


@dataclass(frozen=True)
class ColorResult:
    name: str
    confidence: float
    rgb: tuple[int, int, int]


@dataclass(frozen=True)
class Detection:
    name: str
    score: float
    box: tuple[float, float, float, float]
    color: ColorResult


@dataclass(frozen=True)
class ObjectSnapshot:
    frame_sequence: int = 0
    frame_received_ns: int = 0
    completed_ns: int = 0
    inference_ms: float = 0.0
    detections: tuple[Detection, ...] = ()
    error: str = ""

    @property
    def source_age_ms(self) -> float:
        if not self.frame_received_ns:
            return float("inf")
        return (time.monotonic_ns() - self.frame_received_ns) / 1_000_000.0

    @property
    def result_age_ms(self) -> float:
        if not self.completed_ns:
            return float("inf")
        return (time.monotonic_ns() - self.completed_ns) / 1_000_000.0


@dataclass(frozen=True)
class QrScanResult:
    payloads: tuple[str, ...]
    points: tuple[tuple[tuple[float, float], ...], ...]
    attempts: int
    candidates: int
    decode_errors: int
    elapsed_ms: float


@dataclass
class PipelineStats:
    jpeg_frames: int = 0
    decoded_frames: int = 0
    decode_errors: int = 0
    qr_checked_frames: int = 0
    qr_attempts: int = 0
    qr_candidates: int = 0
    qr_decodes: int = 0
    qr_decode_errors: int = 0
    camera_stalls: int = 0
    yolo_stale_results: int = 0


class MjpegFrameSource:
    """Reconnectable MJPEG reader that emits every complete JPEG in wire order."""

    def __init__(
        self,
        url: str,
        *,
        timeout_s: float = 4.0,
        reconnect_min_s: float = 0.25,
        reconnect_max_s: float = 4.0,
        max_buffer_bytes: int = 4 * 1024 * 1024,
    ) -> None:
        self.url = url
        self.timeout_s = timeout_s
        self.reconnect_min_s = reconnect_min_s
        self.reconnect_max_s = reconnect_max_s
        self.max_buffer_bytes = max_buffer_bytes
        self.stats = SourceStats()
        self._stop = threading.Event()
        self._sequence = 0

    def stop(self) -> None:
        self._stop.set()

    @staticmethod
    def extract_jpegs(buffer: bytearray, max_buffer_bytes: int) -> list[bytes]:
        frames: list[bytes] = []
        while True:
            start = buffer.find(b"\xff\xd8")
            if start < 0:
                if len(buffer) > max_buffer_bytes:
                    del buffer[:-2]
                break
            if start:
                del buffer[:start]
            end = buffer.find(b"\xff\xd9", 2)
            if end < 0:
                if len(buffer) > max_buffer_bytes:
                    del buffer[:-2]
                break
            frames.append(bytes(buffer[: end + 2]))
            del buffer[: end + 2]
        return frames

    @staticmethod
    def _reduce_socket_buffer(response: Any) -> None:
        candidates = (
            getattr(getattr(response, "fp", None), "raw", None),
            getattr(response, "fp", None),
        )
        for candidate in candidates:
            sock = getattr(candidate, "_sock", None)
            if sock is not None:
                try:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 64 * 1024)
                except OSError:
                    pass
                return

    def frames(self) -> Iterator[EncodedFrame]:
        delay_s = self.reconnect_min_s
        while not self._stop.is_set():
            request = urllib.request.Request(
                self.url,
                headers={"Cache-Control": "no-cache", "Pragma": "no-cache"},
            )
            try:
                with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                    self.stats.connections += 1
                    self.stats.last_error = ""
                    self._reduce_socket_buffer(response)
                    buffer = bytearray()
                    delay_s = self.reconnect_min_s
                    while not self._stop.is_set():
                        read_chunk = getattr(response, "read1", response.read)
                        chunk = read_chunk(16 * 1024)
                        if not chunk:
                            raise ConnectionError("camera closed the MJPEG stream")
                        self.stats.bytes_received += len(chunk)
                        buffer.extend(chunk)
                        before = len(buffer)
                        frames = self.extract_jpegs(buffer, self.max_buffer_bytes)
                        if before > self.max_buffer_bytes and not frames:
                            self.stats.oversized_buffers += 1
                        if frames:
                            self.stats.frame_batches += 1
                            self.stats.max_batch_frames = max(self.stats.max_batch_frames, len(frames))
                            if len(frames) > 1:
                                self.stats.burst_batches += 1
                        batch_received_ns = time.monotonic_ns()
                        for jpeg in frames:
                            if len(jpeg) < 128:
                                self.stats.malformed_frames += 1
                                continue
                            self._sequence += 1
                            self.stats.frames_received += 1
                            yield EncodedFrame(self._sequence, batch_received_ns, jpeg)
            except (OSError, TimeoutError, ConnectionError, http.client.HTTPException) as error:
                self.stats.last_error = str(error)
                self.stats.reconnects += 1
                if self._stop.wait(delay_s):
                    return
                delay_s = min(delay_s * 2.0, self.reconnect_max_s)


class FisheyeCorrector:
    def __init__(self, calibration_path: str | Path | None = None) -> None:
        self.enabled = calibration_path is not None
        self._maps: dict[tuple[int, int], tuple[np.ndarray, np.ndarray]] = {}
        self._camera_matrix: np.ndarray | None = None
        self._distortion: np.ndarray | None = None
        self._calibration_size: tuple[int, int] | None = None
        self._balance = 0.0
        if calibration_path is not None:
            self._load(calibration_path)

    def _load(self, path: str | Path) -> None:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        size = data.get("image_size")
        matrix = np.asarray(data.get("camera_matrix"), dtype=np.float64)
        distortion = np.asarray(data.get("distortion_coefficients"), dtype=np.float64)
        if not isinstance(size, list) or len(size) != 2 or min(size) <= 0:
            raise ValueError("fisheye image_size must be [width, height]")
        if matrix.shape != (3, 3):
            raise ValueError("fisheye camera_matrix must be 3x3")
        if distortion.size != 4:
            raise ValueError("fisheye distortion_coefficients must contain four values")
        self._calibration_size = int(size[0]), int(size[1])
        self._camera_matrix = matrix
        self._distortion = distortion.reshape(4, 1)
        self._balance = float(np.clip(data.get("balance", 0.0), 0.0, 1.0))

    def _build_maps(self, size: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
        assert self._camera_matrix is not None
        assert self._distortion is not None
        assert self._calibration_size is not None
        matrix = self._camera_matrix.copy()
        scale_x = size[0] / self._calibration_size[0]
        scale_y = size[1] / self._calibration_size[1]
        matrix[0, 0] *= scale_x
        matrix[0, 2] *= scale_x
        matrix[1, 1] *= scale_y
        matrix[1, 2] *= scale_y
        new_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            matrix, self._distortion, size, np.eye(3), balance=self._balance
        )
        return cv2.fisheye.initUndistortRectifyMap(
            matrix, self._distortion, np.eye(3), new_matrix, size, cv2.CV_16SC2
        )

    def apply(self, image: np.ndarray) -> np.ndarray:
        if not self.enabled:
            return image
        size = image.shape[1], image.shape[0]
        maps = self._maps.get(size)
        if maps is None:
            maps = self._build_maps(size)
            self._maps[size] = maps
        return cv2.remap(
            image,
            maps[0],
            maps[1],
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )


class EveryFrameQrScanner:
    """Scans the full field every frame and rolls costly fallbacks across frames."""

    def __init__(
        self,
        robust: bool = True,
        detector: Any | None = None,
        primary_width: int = 640,
    ) -> None:
        self.robust = robust
        self.detector = detector if detector is not None else cv2.QRCodeDetector()
        self.primary_width = max(160, primary_width)
        self._frame_index = 0
        if hasattr(self.detector, "setEpsX"):
            self.detector.setEpsX(0.20)
            self.detector.setEpsY(0.20)

    @staticmethod
    def _points(
        points: Any,
        offset: tuple[int, int] = (0, 0),
        scale: tuple[float, float] = (1.0, 1.0),
    ) -> list[tuple[tuple[float, float], ...]]:
        if points is None:
            return []
        array = np.asarray(points, dtype=np.float32)
        if array.size == 0:
            return []
        array = array.reshape(-1, 4, 2)
        array[:, :, 0] *= scale[0]
        array[:, :, 1] *= scale[1]
        array[:, :, 0] += offset[0]
        array[:, :, 1] += offset[1]
        return [tuple((float(x), float(y)) for x, y in polygon) for polygon in array]

    def _multi(
        self,
        image: np.ndarray,
        *,
        scale: tuple[float, float] = (1.0, 1.0),
    ) -> tuple[list[str], list[tuple[tuple[float, float], ...]]]:
        result = self.detector.detectAndDecodeMulti(image)
        if len(result) != 4:
            return [], []
        detected, payloads, points, _ = result
        decoded = [str(payload) for payload in payloads if payload]
        return decoded if detected else [], self._points(points, scale=scale)

    def _single(
        self,
        image: np.ndarray,
        *,
        curved: bool = False,
        offset: tuple[int, int] = (0, 0),
        scale: tuple[float, float] = (1.0, 1.0),
    ) -> tuple[list[str], list[tuple[tuple[float, float], ...]]]:
        method = self.detector.detectAndDecodeCurved if curved else self.detector.detectAndDecode
        payload, points, _ = method(image)
        return ([str(payload)] if payload else []), self._points(points, offset, scale)

    @staticmethod
    def _attempt(method: Any, *args: Any, **kwargs: Any) -> tuple[
        list[str], list[tuple[tuple[float, float], ...]], int
    ]:
        try:
            payloads, points = method(*args, **kwargs)
            return payloads, points, 0
        except cv2.error:
            return [], [], 1

    def _primary_image(self, image: np.ndarray) -> tuple[np.ndarray, tuple[float, float]]:
        height, width = image.shape[:2]
        if width <= self.primary_width:
            return image, (1.0, 1.0)
        resized_height = max(1, round(height * self.primary_width / width))
        resized = cv2.resize(image, (self.primary_width, resized_height), interpolation=cv2.INTER_AREA)
        return resized, (width / self.primary_width, height / resized_height)

    def scan(self, image: np.ndarray) -> QrScanResult:
        started = time.perf_counter()
        self._frame_index += 1
        primary, primary_scale = self._primary_image(image)
        attempts = 1
        payloads, points, decode_errors = self._attempt(
            self._multi, primary, scale=primary_scale
        )
        candidates = len(points)
        decode_errors += candidates if candidates and not payloads else 0

        fallback_phase = (self._frame_index - 1) % 12
        if not payloads and self.robust and fallback_phase == 0:
            attempts += 1
            curved_payloads, curved_points, attempt_errors = self._attempt(
                self._single, primary, curved=True, scale=primary_scale
            )
            decode_errors += attempt_errors
            payloads.extend(curved_payloads)
            points.extend(curved_points)
            candidates += len(curved_points)
            if curved_points and not curved_payloads:
                decode_errors += len(curved_points)

        if not payloads and self.robust and fallback_phase == 1:
            grey = cv2.cvtColor(primary, cv2.COLOR_BGR2GRAY)
            contrast = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(grey)
            attempts += 1
            contrast_payloads, contrast_points, attempt_errors = self._attempt(
                self._multi, contrast, scale=primary_scale
            )
            decode_errors += attempt_errors
            payloads.extend(contrast_payloads)
            points.extend(contrast_points)
            candidates += len(contrast_points)
            if contrast_points and not contrast_payloads:
                decode_errors += len(contrast_points)

        if not payloads and self.robust and 2 <= fallback_phase <= 5:
            height, width = image.shape[:2]
            tile_width = int(width * 0.62)
            tile_height = int(height * 0.62)
            offsets = (
                (0, 0),
                (width - tile_width, 0),
                (0, height - tile_height),
                (width - tile_width, height - tile_height),
            )
            left, top = offsets[fallback_phase - 2]
            attempts += 1
            tile = image[top : top + tile_height, left : left + tile_width]
            tile_payloads, tile_points, attempt_errors = self._attempt(
                self._single, tile, offset=(left, top)
            )
            decode_errors += attempt_errors
            payloads.extend(tile_payloads)
            points.extend(tile_points)
            candidates += len(tile_points)
            if tile_points and not tile_payloads:
                decode_errors += len(tile_points)

        return QrScanResult(
            payloads=tuple(dict.fromkeys(payloads)),
            points=tuple(points),
            attempts=attempts,
            candidates=candidates,
            decode_errors=decode_errors,
            elapsed_ms=(time.perf_counter() - started) * 1000.0,
        )


COLOR_NAMES = np.asarray(
    ["black", "white", "grey", "red", "orange", "yellow", "green", "cyan", "blue", "purple", "pink", "brown"]
)


def dominant_color(
    image: np.ndarray,
    box: tuple[float, float, float, float] | None = None,
) -> ColorResult:
    height, width = image.shape[:2]
    if box is None:
        left, top, right, bottom = int(width * 0.15), int(height * 0.15), int(width * 0.85), int(height * 0.85)
    else:
        x1, y1, x2, y2 = box
        margin_x = (x2 - x1) * 0.10
        margin_y = (y2 - y1) * 0.10
        left = int(np.clip((x1 + margin_x) * width, 0, width - 1))
        top = int(np.clip((y1 + margin_y) * height, 0, height - 1))
        right = int(np.clip((x2 - margin_x) * width, left + 1, width))
        bottom = int(np.clip((y2 - margin_y) * height, top + 1, height))
    crop = image[top:bottom, left:right]
    if crop.size == 0:
        return ColorResult("unknown", 0.0, (0, 0, 0))
    if max(crop.shape[:2]) > 180:
        scale = 180.0 / max(crop.shape[:2])
        crop = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    labels = np.full(hue.shape, 2, dtype=np.uint8)
    labels[value < 45] = 0
    achromatic = saturation < 36
    labels[achromatic & (value >= 45) & (value <= 210)] = 2
    labels[achromatic & (value > 210)] = 1
    chromatic = ~achromatic & (value >= 45)
    labels[chromatic & ((hue < 8) | (hue >= 172))] = 3
    orange = chromatic & (hue >= 8) & (hue < 22)
    labels[orange & (value >= 155)] = 4
    labels[orange & (value < 155)] = 11
    labels[chromatic & (hue >= 22) & (hue < 36)] = 5
    labels[chromatic & (hue >= 36) & (hue < 85)] = 6
    labels[chromatic & (hue >= 85) & (hue < 100)] = 7
    labels[chromatic & (hue >= 100) & (hue < 130)] = 8
    labels[chromatic & (hue >= 130) & (hue < 150)] = 9
    labels[chromatic & (hue >= 150) & (hue < 172)] = 10

    counts = np.bincount(labels.ravel(), minlength=len(COLOR_NAMES))
    winner = int(np.argmax(counts))
    winner_mask = labels == winner
    median_bgr = np.median(crop[winner_mask], axis=0) if np.any(winner_mask) else np.zeros(3)
    return ColorResult(
        str(COLOR_NAMES[winner]),
        float(counts[winner] / labels.size),
        (int(median_bgr[2]), int(median_bgr[1]), int(median_bgr[0])),
    )


class LatestFrameSlot:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._frame: FramePacket | None = None
        self.replacements = 0

    def publish(self, frame: FramePacket) -> None:
        with self._condition:
            if self._frame is not None and self._frame.sequence != frame.sequence:
                self.replacements += 1
            self._frame = frame
            self._condition.notify_all()

    def wait_newer(
        self, last_sequence: int, stop: threading.Event, timeout_s: float = 0.25
    ) -> FramePacket | None:
        with self._condition:
            self._condition.wait_for(
                lambda: stop.is_set()
                or (self._frame is not None and self._frame.sequence > last_sequence),
                timeout_s,
            )
            if stop.is_set() or self._frame is None or self._frame.sequence <= last_sequence:
                return None
            return self._frame


class ObjectDetector(Protocol):
    def detect(self, image: np.ndarray) -> tuple[Detection, ...]: ...


class UltralyticsDetector:
    def __init__(
        self,
        model_path: str,
        *,
        confidence: float = 0.35,
        iou: float = 0.45,
        image_size: int = 640,
        device: str | None = None,
        max_detections: int = 20,
    ) -> None:
        from ultralytics import YOLO

        self.model = YOLO(model_path)
        self.confidence = confidence
        self.iou = iou
        self.image_size = image_size
        self.device = device
        self.max_detections = max_detections

    def detect(self, image: np.ndarray) -> tuple[Detection, ...]:
        arguments: dict[str, Any] = {
            "source": image,
            "conf": self.confidence,
            "iou": self.iou,
            "imgsz": self.image_size,
            "max_det": self.max_detections,
            "verbose": False,
        }
        if self.device:
            arguments["device"] = self.device
        result = self.model.predict(**arguments)[0]
        if result.boxes is None:
            return ()
        xyxy = result.boxes.xyxyn.detach().cpu().numpy()
        confidence = result.boxes.conf.detach().cpu().numpy()
        classes = result.boxes.cls.detach().cpu().numpy().astype(int)
        detections: list[Detection] = []
        for coordinates, score, class_id in zip(xyxy, confidence, classes):
            box = tuple(float(np.clip(value, 0.0, 1.0)) for value in coordinates)
            name = str(result.names.get(class_id, class_id))
            detections.append(Detection(name, float(score), box, dominant_color(image, box)))
        detections.sort(key=lambda item: item.score, reverse=True)
        return tuple(detections)


class ObjectWorker(threading.Thread):
    def __init__(
        self,
        slot: LatestFrameSlot,
        detector: ObjectDetector,
        *,
        max_input_age_ms: float = 750.0,
    ) -> None:
        super().__init__(name="latest-frame-yolo", daemon=True)
        self.slot = slot
        self.detector = detector
        self.max_input_age_ms = max_input_age_ms
        self.stop_event = threading.Event()
        self.inferences = 0
        self.stale_inputs = 0
        self.superseded_frames = 0
        self._snapshot = ObjectSnapshot()
        self._lock = threading.Lock()

    def stop(self) -> None:
        self.stop_event.set()

    def snapshot(self) -> ObjectSnapshot:
        with self._lock:
            return self._snapshot

    def run(self) -> None:
        last_sequence = 0
        while not self.stop_event.is_set():
            frame = self.slot.wait_newer(last_sequence, self.stop_event)
            if frame is None:
                continue
            if last_sequence:
                self.superseded_frames += max(0, frame.sequence - last_sequence - 1)
            last_sequence = frame.sequence
            if frame.age_ms > self.max_input_age_ms:
                self.stale_inputs += 1
                continue
            started = time.perf_counter()
            error = ""
            detections: tuple[Detection, ...] = ()
            try:
                detections = self.detector.detect(frame.image)
            except Exception as exception:
                error = f"{type(exception).__name__}: {exception}"
            completed = time.monotonic_ns()
            snapshot = ObjectSnapshot(
                frame_sequence=frame.sequence,
                frame_received_ns=frame.received_ns,
                completed_ns=completed,
                inference_ms=(time.perf_counter() - started) * 1000.0,
                detections=detections,
                error=error,
            )
            with self._lock:
                self._snapshot = snapshot
            self.inferences += 1


def object_snapshot_to_dict(snapshot: ObjectSnapshot, fresh_ms: float) -> dict[str, Any]:
    source_age_ms = snapshot.source_age_ms
    result_age_ms = snapshot.result_age_ms
    return {
        "fresh": source_age_ms <= fresh_ms and not snapshot.error,
        "age_ms": round(source_age_ms, 1) if np.isfinite(source_age_ms) else None,
        "source_age_ms": round(source_age_ms, 1) if np.isfinite(source_age_ms) else None,
        "result_age_ms": round(result_age_ms, 1) if np.isfinite(result_age_ms) else None,
        "frame_sequence": snapshot.frame_sequence,
        "inference_ms": round(snapshot.inference_ms, 1),
        "error": snapshot.error,
        "objects": [asdict(detection) for detection in snapshot.detections],
    }
