from __future__ import annotations

import json
import http.client
import threading
import tempfile
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from vision_core import (
    EveryFrameQrScanner,
    FisheyeCorrector,
    FramePacket,
    LatestFrameSlot,
    MjpegFrameSource,
    ObjectSnapshot,
    ObjectWorker,
    dominant_color,
    object_snapshot_to_dict,
)


class FakeQrDetector:
    def __init__(self) -> None:
        self.calls = 0

    def setEpsX(self, _value):
        return self

    def setEpsY(self, _value):
        return self

    def detectAndDecodeMulti(self, _image):
        self.calls += 1
        points = np.asarray([[[1, 1], [8, 1], [8, 8], [1, 8]]], dtype=np.float32)
        return True, ("TEST-QR",), points, ()

    def detectAndDecode(self, _image):
        return "", None, None

    def detectAndDecodeCurved(self, _image):
        return "", None, None


class FlakyQrDetector(FakeQrDetector):
    def detectAndDecodeMulti(self, image):
        if self.calls == 0:
            self.calls += 1
            raise cv2.error("temporary QR decoder failure")
        return super().detectAndDecodeMulti(image)


class BlockingObjectDetector:
    def __init__(self) -> None:
        self.started = threading.Event()
        self.release = threading.Event()

    def detect(self, _image):
        self.started.set()
        self.release.wait(1.0)
        return ()


class BufferedResponse:
    def __init__(self, payload: bytes) -> None:
        self.payload = payload
        self.read1_calls = 0

    def __enter__(self):
        return self

    def __exit__(self, _kind, _value, _traceback):
        return False

    def read1(self, _size: int) -> bytes:
        self.read1_calls += 1
        payload, self.payload = self.payload, b""
        return payload

    def read(self, _size: int) -> bytes:
        raise AssertionError("low-latency source must use read1 when available")


class TruncatedResponse(BufferedResponse):
    def read1(self, size: int) -> bytes:
        if self.payload:
            return super().read1(size)
        raise http.client.IncompleteRead(b"partial")


class VisionCoreTests(unittest.TestCase):
    @staticmethod
    def make_qr_jpeg(payload: str, scale: int = 14) -> bytes:
        qr = cv2.QRCodeEncoder_create().encode(payload)
        qr = cv2.copyMakeBorder(qr, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=255)
        qr = cv2.resize(qr, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        image = np.full((720, 1280, 3), 255, dtype=np.uint8)
        qr_bgr = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
        top = (image.shape[0] - qr_bgr.shape[0]) // 2
        left = (image.shape[1] - qr_bgr.shape[1]) // 2
        image[top : top + qr_bgr.shape[0], left : left + qr_bgr.shape[1]] = qr_bgr
        encoded, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not encoded:
            raise AssertionError("test JPEG encoding failed")
        return jpeg.tobytes()

    def test_mjpeg_parser_preserves_every_complete_frame(self):
        first = b"\xff\xd8" + b"a" * 128 + b"\xff\xd9"
        second = b"\xff\xd8" + b"b" * 128 + b"\xff\xd9"
        buffer = bytearray(b"header\r\n" + first + b"boundary" + second + b"tail")
        frames = MjpegFrameSource.extract_jpegs(buffer, 1024)
        self.assertEqual(frames, [first, second])
        self.assertEqual(buffer, bytearray(b"tail"))

    def test_socket_burst_frames_keep_one_arrival_timestamp(self):
        first = b"\xff\xd8" + b"a" * 128 + b"\xff\xd9"
        second = b"\xff\xd8" + b"b" * 128 + b"\xff\xd9"
        response = BufferedResponse(first + second)
        source = MjpegFrameSource("http://camera.invalid/stream")
        with patch("vision_core.urllib.request.urlopen", return_value=response):
            iterator = source.frames()
            frame_one = next(iterator)
            time.sleep(0.02)
            frame_two = next(iterator)
            source.stop()
            iterator.close()
        self.assertEqual(response.read1_calls, 1)
        self.assertEqual(frame_one.received_ns, frame_two.received_ns)
        self.assertEqual(source.stats.frame_batches, 1)
        self.assertEqual(source.stats.burst_batches, 1)
        self.assertEqual(source.stats.max_batch_frames, 2)

    def test_flaky_mjpeg_reconnect_scans_every_recovered_frame(self):
        jpeg = self.make_qr_jpeg("FLAKY-QR")
        batches = [[jpeg, jpeg], [jpeg, jpeg]]
        batches_lock = threading.Lock()

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, _format, *_arguments):
                pass

            def do_GET(self):
                with batches_lock:
                    batch = batches.pop(0) if batches else []
                if not batch:
                    self.send_error(503)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                for frame in batch:
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(frame)).encode("ascii")
                        + b"\r\n\r\n"
                        + frame
                        + b"\r\n"
                    )
                self.wfile.flush()

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        server.daemon_threads = True
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        source = MjpegFrameSource(
            f"http://127.0.0.1:{server.server_port}/stream",
            timeout_s=1.0,
            reconnect_min_s=0.01,
            reconnect_max_s=0.05,
        )
        scanner = EveryFrameQrScanner(robust=False)
        frames = []
        try:
            iterator = source.frames()
            while len(frames) < 4:
                encoded = next(iterator)
                image = cv2.imdecode(np.frombuffer(encoded.jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                self.assertIsNotNone(image)
                result = scanner.scan(image)
                self.assertEqual(result.payloads, ("FLAKY-QR",))
                frames.append(encoded)
            source.stop()
            iterator.close()
        finally:
            source.stop()
            server.shutdown()
            server.server_close()
            server_thread.join(timeout=1.0)
        self.assertEqual([frame.sequence for frame in frames], [1, 2, 3, 4])
        self.assertEqual(source.stats.frames_received, 4)
        self.assertGreaterEqual(source.stats.connections, 2)
        self.assertGreaterEqual(source.stats.reconnects, 1)

    def test_incomplete_http_read_reconnects_without_losing_recovered_frame(self):
        jpeg = self.make_qr_jpeg("RECOVERED-QR")
        responses = [TruncatedResponse(jpeg), BufferedResponse(jpeg)]
        source = MjpegFrameSource(
            "http://camera.invalid/stream", reconnect_min_s=0.001, reconnect_max_s=0.001
        )
        with patch("vision_core.urllib.request.urlopen", side_effect=responses):
            iterator = source.frames()
            first = next(iterator)
            second = next(iterator)
            source.stop()
            iterator.close()
        self.assertEqual([first.sequence, second.sequence], [1, 2])
        self.assertEqual(source.stats.connections, 2)
        self.assertEqual(source.stats.reconnects, 1)

    def test_qr_scanner_checks_each_supplied_frame(self):
        detector = FakeQrDetector()
        scanner = EveryFrameQrScanner(robust=True, detector=detector)
        image = np.zeros((20, 20, 3), dtype=np.uint8)
        for _ in range(7):
            result = scanner.scan(image)
            self.assertEqual(result.payloads, ("TEST-QR",))
            self.assertGreaterEqual(result.attempts, 1)
        self.assertEqual(detector.calls, 7)

    def test_qr_decoder_exception_is_contained_to_one_checked_frame(self):
        detector = FlakyQrDetector()
        scanner = EveryFrameQrScanner(robust=False, detector=detector)
        image = np.zeros((20, 20, 3), dtype=np.uint8)
        failed = scanner.scan(image)
        recovered = scanner.scan(image)
        self.assertEqual(failed.payloads, ())
        self.assertEqual(failed.decode_errors, 1)
        self.assertEqual(recovered.payloads, ("TEST-QR",))
        self.assertEqual(detector.calls, 2)

    def test_latest_frame_slot_never_returns_backlog(self):
        slot = LatestFrameSlot()
        image = np.zeros((2, 2, 3), dtype=np.uint8)
        for sequence in range(1, 6):
            slot.publish(FramePacket(sequence, 1, image))
        newest = slot.wait_newer(0, threading.Event(), timeout_s=0.01)
        self.assertIsNotNone(newest)
        self.assertEqual(newest.sequence, 5)
        self.assertEqual(slot.replacements, 4)

    def test_slow_yolo_worker_skips_superseded_frames(self):
        slot = LatestFrameSlot()
        detector = BlockingObjectDetector()
        worker = ObjectWorker(slot, detector)
        image = np.zeros((2, 2, 3), dtype=np.uint8)
        worker.start()
        try:
            slot.publish(FramePacket(1, time.monotonic_ns(), image))
            self.assertTrue(detector.started.wait(1.0))
            for sequence in range(2, 6):
                slot.publish(FramePacket(sequence, time.monotonic_ns(), image))
            detector.release.set()
            deadline = time.monotonic() + 1.0
            while worker.snapshot().frame_sequence != 5 and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(worker.snapshot().frame_sequence, 5)
            self.assertEqual(worker.superseded_frames, 3)
        finally:
            detector.release.set()
            worker.stop()
            worker.join(timeout=1.0)

    def test_yolo_freshness_uses_source_frame_age(self):
        now = time.monotonic_ns()
        snapshot = ObjectSnapshot(
            frame_sequence=7,
            frame_received_ns=now - 2_000_000_000,
            completed_ns=now,
        )
        serialized = object_snapshot_to_dict(snapshot, fresh_ms=500.0)
        self.assertFalse(serialized["fresh"])
        self.assertGreaterEqual(serialized["source_age_ms"], 1900.0)
        self.assertLess(serialized["result_age_ms"], 100.0)

    def test_real_opencv_qr_decode(self):
        qr = cv2.QRCodeEncoder_create().encode("REAL-QR")
        qr = cv2.copyMakeBorder(qr, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=255)
        qr = cv2.resize(qr, None, fx=8, fy=8, interpolation=cv2.INTER_NEAREST)
        image = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
        self.assertEqual(EveryFrameQrScanner(robust=False).scan(image).payloads, ("REAL-QR",))

    def test_high_detail_fallback_recovers_small_qr(self):
        jpeg = self.make_qr_jpeg("SMALL-QR", scale=3)
        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(EveryFrameQrScanner(robust=False).scan(image).payloads, ())
        scanner = EveryFrameQrScanner(robust=True)
        results = [scanner.scan(image).payloads for _ in range(3)]
        self.assertEqual(results[2], ("SMALL-QR",))

    def test_fisheye_calibration_is_loaded_and_applied(self):
        calibration = {
            "image_size": [64, 48],
            "camera_matrix": [[50.0, 0.0, 32.0], [0.0, 50.0, 24.0], [0.0, 0.0, 1.0]],
            "distortion_coefficients": [0.0, 0.0, 0.0, 0.0],
            "balance": 0.0,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fisheye.json"
            path.write_text(json.dumps(calibration), encoding="utf-8")
            corrected = FisheyeCorrector(path).apply(np.zeros((48, 64, 3), dtype=np.uint8))
        self.assertEqual(corrected.shape, (48, 64, 3))

    def test_basic_color_classification(self):
        red = np.full((80, 80, 3), (0, 0, 255), dtype=np.uint8)
        white = np.full((80, 80, 3), (245, 245, 245), dtype=np.uint8)
        brown_hsv = np.full((80, 80, 3), (15, 210, 120), dtype=np.uint8)
        brown = cv2.cvtColor(brown_hsv, cv2.COLOR_HSV2BGR)
        self.assertEqual(dominant_color(red).name, "red")
        self.assertEqual(dominant_color(white).name, "white")
        self.assertEqual(dominant_color(brown).name, "brown")


if __name__ == "__main__":
    unittest.main()
