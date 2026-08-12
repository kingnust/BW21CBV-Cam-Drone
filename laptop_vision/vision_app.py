from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from dataclasses import asdict
from pathlib import Path

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
    PipelineStats,
    UltralyticsDetector,
    object_snapshot_to_dict,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Low-latency BW21 QR, object, and color vision")
    parser.add_argument("--stream", default="http://192.168.1.1:81/stream")
    parser.add_argument("--model", default="yolo26n.pt", help="Ultralytics model name or local path")
    parser.add_argument("--device", default=None, help="Ultralytics device such as cpu or 0")
    parser.add_argument("--fisheye", type=Path, default=None, help="JSON made by calibrate_fisheye.py")
    parser.add_argument("--output", type=Path, default=Path("vision_events.jsonl"))
    parser.add_argument("--no-yolo", action="store_true")
    parser.add_argument("--fast-qr", action="store_true", help="Disable rolling robust QR fallbacks")
    parser.add_argument("--qr-width", type=int, default=640, help="Every-frame QR analysis width")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--max-input-age-ms", type=float, default=750.0)
    parser.add_argument("--object-fresh-ms", type=float, default=1500.0)
    parser.add_argument("--stall-ms", type=float, default=500.0)
    return parser.parse_args()


def draw_overlay(image: np.ndarray, qr_result, objects: ObjectSnapshot, object_fresh_ms: float) -> np.ndarray:
    output = image.copy()
    for polygon in qr_result.points:
        points = np.asarray(polygon, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(output, [points], True, (60, 220, 90), 2)
    if objects.source_age_ms <= object_fresh_ms:
        height, width = output.shape[:2]
        for detection in objects.detections:
            x1, y1, x2, y2 = detection.box
            left, top = int(x1 * width), int(y1 * height)
            right, bottom = int(x2 * width), int(y2 * height)
            cv2.rectangle(output, (left, top), (right, bottom), (40, 190, 255), 2)
            label = f"{detection.name} {detection.score:.0%} {detection.color.name}"
            cv2.putText(output, label, (left, max(18, top - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 190, 255), 1, cv2.LINE_AA)
    qr_text = " | ".join(qr_result.payloads) if qr_result.payloads else "QR --"
    cv2.putText(output, qr_text[:100], (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 220, 90), 2, cv2.LINE_AA)
    return output


def main() -> int:
    args = parse_args()
    if not 0.0 < args.confidence <= 1.0 or not 0.0 < args.iou <= 1.0:
        raise SystemExit("--confidence and --iou must be in (0, 1]")

    source = MjpegFrameSource(args.stream)
    corrector = FisheyeCorrector(args.fisheye)
    qr_scanner = EveryFrameQrScanner(robust=not args.fast_qr, primary_width=args.qr_width)
    stats = PipelineStats()
    slot = LatestFrameSlot()
    worker: ObjectWorker | None = None
    if not args.no_yolo:
        detector = UltralyticsDetector(
            args.model,
            confidence=args.confidence,
            iou=args.iou,
            image_size=args.image_size,
            device=args.device,
        )
        worker = ObjectWorker(slot, detector, max_input_age_ms=args.max_input_age_ms)
        worker.start()

    stopping = False

    def stop_handler(_signum, _frame) -> None:
        nonlocal stopping
        stopping = True
        source.stop()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    previous_received_ns = 0
    last_status_ns = 0
    last_payloads: tuple[str, ...] = ()
    object_was_stale = False
    output = args.output.open("a", encoding="utf-8", buffering=1)
    try:
        for encoded in source.frames():
            if stopping:
                break
            stats.jpeg_frames += 1
            encoded_array = np.frombuffer(encoded.jpeg, dtype=np.uint8)
            image = cv2.imdecode(encoded_array, cv2.IMREAD_COLOR)
            if image is None:
                stats.decode_errors += 1
                continue
            stats.decoded_frames += 1
            image = corrector.apply(image)

            qr_result = qr_scanner.scan(image)
            stats.qr_checked_frames += 1
            stats.qr_attempts += qr_result.attempts
            stats.qr_candidates += qr_result.candidates
            stats.qr_decodes += len(qr_result.payloads)
            stats.qr_decode_errors += qr_result.decode_errors

            if previous_received_ns and (encoded.received_ns - previous_received_ns) / 1_000_000.0 > args.stall_ms:
                stats.camera_stalls += 1
            previous_received_ns = encoded.received_ns

            packet = FramePacket(encoded.sequence, encoded.received_ns, image)
            slot.publish(packet)
            objects = worker.snapshot() if worker else ObjectSnapshot()
            object_is_stale = bool(objects.frame_sequence) and objects.source_age_ms > args.object_fresh_ms
            if object_is_stale and not object_was_stale:
                stats.yolo_stale_results += 1
            object_was_stale = object_is_stale
            now_ns = time.monotonic_ns()
            changed_qr = bool(qr_result.payloads) and qr_result.payloads != last_payloads
            periodic = now_ns - last_status_ns >= 1_000_000_000
            if changed_qr or periodic:
                event = {
                    "time_unix": time.time(),
                    "backend": "laptop",
                    "frame_sequence": encoded.sequence,
                    "frame_age_ms": round(packet.age_ms, 1),
                    "fisheye_corrected": corrector.enabled,
                    "qr": {
                        "checked": True,
                        "payloads": list(qr_result.payloads),
                        "attempts": qr_result.attempts,
                        "candidates": qr_result.candidates,
                        "decode_errors": qr_result.decode_errors,
                        "scan_ms": round(qr_result.elapsed_ms, 1),
                    },
                    "object_detection": object_snapshot_to_dict(objects, args.object_fresh_ms),
                    "stats": asdict(stats),
                    "source": asdict(source.stats),
                    "yolo_latest_slot_replacements": slot.replacements,
                    "yolo_superseded_frames": worker.superseded_frames if worker else 0,
                    "yolo_stale_inputs": worker.stale_inputs if worker else 0,
                }
                output.write(json.dumps(event, ensure_ascii=True) + "\n")
                print(
                    f"frame={encoded.sequence} qr={list(qr_result.payloads)} "
                    f"objects={len(objects.detections)} "
                    f"qr_ms={qr_result.elapsed_ms:.1f}",
                    flush=True,
                )
                last_status_ns = now_ns
                last_payloads = qr_result.payloads

            if not args.no_display:
                cv2.imshow("BW21 Laptop Vision", draw_overlay(image, qr_result, objects, args.object_fresh_ms))
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    finally:
        source.stop()
        if worker:
            worker.stop()
            worker.join(timeout=2.0)
        output.close()
        cv2.destroyAllWindows()

    if stats.decoded_frames != stats.qr_checked_frames:
        print("ERROR: not every decoded frame reached QR scanning", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
