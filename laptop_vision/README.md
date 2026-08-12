# BW21 Laptop Vision

This backend reads the BW21 MJPEG stream without a queued video decoder. Every complete JPEG is decoded and passed through QR detection synchronously. YOLO runs independently from a one-frame latest slot, so slow inference drops superseded object frames instead of accumulating latency.

## Setup

Use a short virtual-environment path on Windows. PyTorch contains deeply nested headers that can exceed the legacy path limit inside a OneDrive project folder.

```powershell
py -3.13 -m venv "$env:USERPROFILE\.venvs\bw21-vision"
$python = "$env:USERPROFILE\.venvs\bw21-vision\Scripts\python.exe"
& $python -m pip install -r requirements.txt
```

Connect to `BW21-CAM-TEST`, then run:

```powershell
& $python vision_app.py
```

Useful modes:

```powershell
# QR and color only
& $python vision_app.py --no-yolo

# Calibrated fisheye correction
& $python vision_app.py --fisheye fisheye_calibration.json

# Custom trained YOLO model
& $python vision_app.py --model C:\models\drone-items.pt
```

The default pretrained model recognizes common COCO objects. Detecting project-specific classes such as dangerous items, custom packages, or drone landing markers requires a trained model supplied through `--model`.

## Fisheye Calibration

Capture at least 8 sharp checkerboard images across the center, edges, distances, and tilts of the camera view. Keep the stream resolution unchanged while calibrating and operating.

```powershell
& $python calibrate_fisheye.py "calibration\*.jpg" --columns 9 --rows 6 --square-mm 20
```

Every decoded frame gets a half-resolution full-field QR pass. Curved, contrast-enhanced, and high-detail quadrant fallbacks rotate across twelve frames to improve difficult and fisheye QR reads without one long scan blocking the live stream. General objects are reported with color measured inside each detection; scene-wide color is intentionally omitted.

`vision_events.jsonl` records QR attempts, detections, colors, frame ages, stalls, reconnects, and superseded YOLO frames. The program exits with an error if any successfully decoded MJPEG frame bypasses QR scanning.
