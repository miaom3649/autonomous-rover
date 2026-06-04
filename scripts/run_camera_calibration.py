#!/usr/bin/env python3
# Hardware diagnostic script — direct SDK access intentional.
# Not a ROS node. Run on physical hardware only.
"""
Run OpenCV camera calibration on frames captured by capture_calibration_frames.py.

Reads frames from /tmp/calibration_frames/, computes intrinsics, and writes:
  config/camera.yaml  — ORB-SLAM3 format

Usage:
    python3 scripts/run_camera_calibration.py --size 8x6 --square 0.009
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml


FRAMES_DIR = Path("/tmp/calibration_frames")
OUTPUT_YAML = Path("config/camera.yaml")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--size", default="8x6",
                   help="Inner corner count as COLSxROWS (default: 8x6)")
    p.add_argument("--square", type=float, default=0.025,
                   help="Square side length in metres (default: 0.025)")
    p.add_argument("--scale", type=float, default=1.0,
                   help="Scale factor applied to intrinsics and image size before saving "
                        "(e.g. 0.5 to convert 640x480 calibration frames to 320x240 output)")
    return p.parse_args()


def collect_points(
    frames: list[Path], board_size: tuple[int, int], square_m: float
) -> tuple[list, list, tuple[int, int]]:
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= square_m

    obj_points: list = []
    img_points: list = []
    img_shape: tuple[int, int] = (0, 0)
    good = 0

    for path in frames:
        img = cv2.imread(str(path))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape  # (h, w)

        ret, corners = cv2.findChessboardCorners(gray, board_size, None)
        if not ret:
            print(f"  [skip] {path.name} — checkerboard not detected")
            continue

        corners_refined = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        obj_points.append(objp)
        img_points.append(corners_refined)
        good += 1
        print(f"  [ok]   {path.name}")

    print(f"\n{good}/{len(frames)} frames usable.")
    return obj_points, img_points, img_shape


def calibrate(
    obj_points: list, img_points: list, img_shape: tuple[int, int]
) -> tuple[np.ndarray, np.ndarray, float, list, list]:
    h, w = img_shape
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (w, h), None, None
    )
    return mtx, dist, ret, rvecs, tvecs


def per_frame_rms(
    obj_points: list,
    img_points: list,
    rvecs: list,
    tvecs: list,
    mtx: np.ndarray,
    dist: np.ndarray,
) -> list[float]:
    errors = []
    for obj, img, rvec, tvec in zip(obj_points, img_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, mtx, dist)
        err = np.sqrt(np.mean((img - projected) ** 2))
        errors.append(float(err))
    return errors


def reject_outliers(
    obj_points: list,
    img_points: list,
    errors: list[float],
    threshold: float,
) -> tuple[list, list, list[int]]:
    kept_obj, kept_img, rejected_idx = [], [], []
    for i, (obj, img, err) in enumerate(zip(obj_points, img_points, errors)):
        if err <= threshold:
            kept_obj.append(obj)
            kept_img.append(img)
        else:
            rejected_idx.append(i)
    return kept_obj, kept_img, rejected_idx


def write_yaml(
    path: Path, mtx: np.ndarray, dist: np.ndarray, w: int, h: int, rms: float
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    # ORB-SLAM3 PinHole format
    data = {
        "Camera.type": "PinHole",
        "Camera.fx": float(mtx[0, 0]),
        "Camera.fy": float(mtx[1, 1]),
        "Camera.cx": float(mtx[0, 2]),
        "Camera.cy": float(mtx[1, 2]),
        "Camera.k1": float(dist[0, 0]),
        "Camera.k2": float(dist[0, 1]),
        "Camera.p1": float(dist[0, 2]),
        "Camera.p2": float(dist[0, 3]),
        "Camera.width": w,
        "Camera.height": h,
        "Camera.fps": 15.0,
        "Camera.RGB": 1,
        "# RMS reprojection error": rms,
    }

    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


def main() -> None:
    args = parse_args()

    cols, rows = (int(x) for x in args.size.lower().split("x"))
    board_size = (cols, rows)

    frames = sorted(FRAMES_DIR.glob("*.jpg"))
    if not frames:
        print(f"No frames found in {FRAMES_DIR}. Run capture_calibration_frames.py first.")
        sys.exit(1)

    print(f"Board: {cols}x{rows} inner corners, square={args.square*1000:.1f} mm")
    print(f"Processing {len(frames)} frames...\n")

    obj_pts, img_pts, img_shape = collect_points(frames, board_size, args.square)

    if len(obj_pts) < 5:
        print("Not enough usable frames (need at least 5). Capture more frames.")
        sys.exit(1)

    print("Running initial calibration...")
    mtx, dist, rms, rvecs, tvecs = calibrate(obj_pts, img_pts, img_shape)
    print(f"Initial RMS: {rms:.4f} px")

    # Iteratively reject frames whose per-frame error exceeds 2× the median.
    for iteration in range(5):
        errors = per_frame_rms(obj_pts, img_pts, rvecs, tvecs, mtx, dist)
        threshold = 2.0 * float(np.median(errors))
        obj_pts, img_pts, rejected = reject_outliers(obj_pts, img_pts, errors, threshold)
        if not rejected:
            break
        print(f"Iteration {iteration + 1}: rejected {len(rejected)} frame(s) "
              f"with error > {threshold:.2f} px — {len(obj_pts)} remaining")
        if len(obj_pts) < 5:
            print("Not enough frames left after filtering. Stopping early.")
            break
        mtx, dist, rms, rvecs, tvecs = calibrate(obj_pts, img_pts, img_shape)
        print(f"  RMS after rejection: {rms:.4f} px")

    h, w = img_shape
    if args.scale != 1.0:
        mtx = mtx.copy()
        mtx[0, 0] *= args.scale   # fx
        mtx[1, 1] *= args.scale   # fy
        mtx[0, 2] *= args.scale   # cx
        mtx[1, 2] *= args.scale   # cy
        w = round(w * args.scale)
        h = round(h * args.scale)
        print(f"\nScaled intrinsics by {args.scale} → output size {w}x{h}")
    write_yaml(OUTPUT_YAML, mtx, dist, w, h, rms)

    print(f"\nCalibration complete — RMS reprojection error: {rms:.4f} px")
    print(f"(Good calibration is typically < 1.0 px)")
    print(f"\nSaved to: {OUTPUT_YAML}")
    print(f"  fx={mtx[0,0]:.2f}  fy={mtx[1,1]:.2f}")
    print(f"  cx={mtx[0,2]:.2f}  cy={mtx[1,2]:.2f}")
    print(f"  k1={dist[0,0]:.5f}  k2={dist[0,1]:.5f}")


if __name__ == "__main__":
    main()
