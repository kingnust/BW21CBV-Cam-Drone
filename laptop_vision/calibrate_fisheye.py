from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate the BW21 fisheye lens from checkerboard photos")
    parser.add_argument("images", nargs="+", help="Image paths or glob patterns")
    parser.add_argument("--columns", type=int, default=9, help="Checkerboard inner-corner columns")
    parser.add_argument("--rows", type=int, default=6, help="Checkerboard inner-corner rows")
    parser.add_argument("--square-mm", type=float, default=20.0)
    parser.add_argument("--balance", type=float, default=0.0)
    parser.add_argument("--output", type=Path, default=Path("fisheye_calibration.json"))
    return parser.parse_args()


def expand_images(patterns: list[str]) -> list[Path]:
    paths: set[Path] = set()
    for pattern in patterns:
        matches = glob.glob(pattern)
        if matches:
            paths.update(Path(match) for match in matches)
        elif Path(pattern).is_file():
            paths.add(Path(pattern))
    return sorted(paths)


def main() -> int:
    args = parse_args()
    if args.columns < 3 or args.rows < 3 or args.square_mm <= 0:
        raise SystemExit("checkerboard dimensions and square size must be positive")
    paths = expand_images(args.images)
    if not paths:
        raise SystemExit("no calibration images matched")

    pattern_size = args.columns, args.rows
    coordinates = np.zeros((1, args.columns * args.rows, 3), np.float64)
    coordinates[0, :, :2] = np.mgrid[0 : args.columns, 0 : args.rows].T.reshape(-1, 2)
    coordinates *= args.square_mm
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None
    used: list[str] = []

    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        current_size = image.shape[1], image.shape[0]
        if image_size is None:
            image_size = current_size
        if current_size != image_size:
            raise SystemExit(f"all images must have one resolution; {path} differs")
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(
            grey,
            pattern_size,
            flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY,
        )
        if found:
            object_points.append(coordinates.copy())
            image_points.append(corners.reshape(1, -1, 2).astype(np.float64))
            used.append(str(path))

    if image_size is None or len(image_points) < 8:
        raise SystemExit(f"need at least 8 usable checkerboard views; found {len(image_points)}")

    camera_matrix = np.zeros((3, 3), dtype=np.float64)
    distortion = np.zeros((4, 1), dtype=np.float64)
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_FIX_SKEW
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7)
    rms, camera_matrix, distortion, _, _ = cv2.fisheye.calibrate(
        object_points,
        image_points,
        image_size,
        camera_matrix,
        distortion,
        None,
        None,
        flags,
        criteria,
    )
    result = {
        "schema": 1,
        "model": "opencv_fisheye",
        "image_size": list(image_size),
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": distortion.reshape(-1).tolist(),
        "balance": float(np.clip(args.balance, 0.0, 1.0)),
        "rms_error": float(rms),
        "checkerboard": {
            "inner_corners": [args.columns, args.rows],
            "square_mm": args.square_mm,
            "images_used": len(used),
        },
        "source_images": used,
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"Saved {args.output} using {len(used)} images; RMS error={rms:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
