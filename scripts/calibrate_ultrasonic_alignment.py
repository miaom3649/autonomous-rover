"""
Ultrasonic-to-camera alignment calibration.

One-time, one-shot-per-sample calibration tool. Answers: "which part of the
camera frame is the ultrasonic sensor actually measuring?" — needed because
depth_bridge_node.py currently just assumes it's the exact center of the
frame, which may not be true (undocumented mounting geometry, camera held
at a non-zero pan/tilt).

Setup: place a solid, brightly-colored rectangular object (default: red)
directly ahead of the rover, within both the ultrasonic sensor's and
camera's range, against a plain background so it's the only thing that
color in frame.

Method: the object's true position in the frame is found by color
thresholding on the plain RGB image — a deterministic, traditional
computer-vision technique, completely independent of the AI depth model.
This avoids the circular trap of using the AI's own (unreliable) depth
values to figure out where to trust the AI's depth values. Once we know
where the object really is, we read the AI's depth estimate at that exact
spot and compare it to the ultrasonic reading.

Requires drive_node already running separately (it's the ultrasonic data
source — see depth_bridge_node.py's docstring for why it isn't started
from here). camera_node is auto-started if nothing is publishing yet, same
as scripts/depth_viewer.py.

Usage (on Pi):
    source /opt/ros/humble/setup.bash
    source ~/dev/autonomous-rover/install/setup.bash
    python3 scripts/calibrate_ultrasonic_alignment.py --url http://<windows-ip>:8765/depth
"""
import argparse
import time

import cv2
import numpy as np
import rclpy

from depth_viewer import (
    _LiveFeed,
    _camera_topic_has_publisher,
    _post_and_decode,
    _start_camera_node,
)

# Red wraps around hue 0 in OpenCV's HSV (0-179), so two bands are needed.
DEFAULT_HSV_RANGES = [
    ((0, 80, 60), (10, 255, 255)),
    ((170, 80, 60), (179, 255, 255)),
]
MIN_OBJECT_AREA = 400  # pixels; filters out small noise blobs
ULTRASONIC_MAX_AGE = 1.0
SAMPLE_INTERVAL_S = 0.5


def _detect_object(bgr: np.ndarray, hsv_ranges: list) -> tuple | None:
    """Find the largest blob matching `hsv_ranges`. Returns (cx, cy, mask) or None."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in hsv_ranges:
        mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_OBJECT_AREA:
        return None

    obj_mask = np.zeros_like(mask)
    cv2.drawContours(obj_mask, [largest], -1, 255, -1)
    moments = cv2.moments(largest)
    if moments["m00"] == 0:
        return None
    cx = moments["m10"] / moments["m00"]
    cy = moments["m01"] / moments["m00"]
    return cx, cy, obj_mask


def _take_sample(node: _LiveFeed, url: str, timeout_s: float, hsv_ranges: list) -> dict | None:
    frame = node.frame
    us_range = node.ultrasonic_range
    us_age = time.monotonic() - node.ultrasonic_stamp
    if frame is None:
        print("  skip: no camera frame yet")
        return None
    if us_range is None or us_age > ULTRASONIC_MAX_AGE:
        print("  skip: no fresh ultrasonic reading (is drive_node running?)")
        return None
    if us_range <= node.ultrasonic_min_range or us_range >= node.ultrasonic_max_range:
        print(f"  skip: ultrasonic reading {us_range:.2f}m at sensor limits (no valid echo?)")
        return None

    detected = _detect_object(frame, hsv_ranges)
    if detected is None:
        print("  skip: red object not found in frame")
        return None
    cx, cy, obj_mask = detected

    try:
        depth, _ = _post_and_decode(frame, url, timeout_s)
    except Exception as exc:
        print(f"  skip: depth server request failed: {exc}")
        return None

    region = depth[obj_mask > 0]
    valid = region[(region > 0) & np.isfinite(region)]
    if valid.size < obj_mask.sum() / 255 * 0.1:
        print("  skip: AI depth mostly invalid over the detected object")
        return None
    ai_estimate = float(valid.mean())

    h, w = depth.shape
    row_frac = cy / h
    col_frac = cx / w
    print(f"  pos=({col_frac:.1%} w, {row_frac:.1%} h)  "
          f"AI={ai_estimate:.2f}m  ultrasonic={us_range:.2f}m  "
          f"diff={abs(ai_estimate - us_range):.2f}m")
    return {"row_frac": row_frac, "col_frac": col_frac,
            "ai": ai_estimate, "ultrasonic": us_range}


def main() -> None:
    parser = argparse.ArgumentParser(description="Ultrasonic-to-camera alignment calibration")
    parser.add_argument("--url", default="http://192.168.1.151:8765/depth",
                        help="Depth server URL")
    parser.add_argument("--topic", default="/rover/camera/image_raw",
                        help="ROS2 image topic to grab from")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="HTTP request timeout in seconds")
    parser.add_argument("--samples", type=int, default=10,
                        help="Number of successful samples to collect")
    parser.add_argument("--no-camera", action="store_true",
                        help="Don't auto-start camera_node even if the topic has no publisher")
    args = parser.parse_args()

    print("=== Ultrasonic-to-Camera Alignment Calibration ===")
    print("Place a solid red rectangular object directly ahead, in range of")
    print("both the ultrasonic sensor and the camera, against a plain background.")
    print()

    started_camera_proc = None
    if not args.no_camera and not _camera_topic_has_publisher(args.topic):
        started_camera_proc = _start_camera_node()
        time.sleep(3.0)  # let libcamera finish initializing before we start sampling

    try:
        rclpy.init()
        node = _LiveFeed(args.topic)

        results = []
        attempts = 0
        max_attempts = args.samples * 4
        while len(results) < args.samples and attempts < max_attempts:
            attempts += 1
            rclpy.spin_once(node, timeout_sec=0.2)
            print(f"[{len(results) + 1}/{args.samples}]", end=" ")
            sample = _take_sample(node, args.url, args.timeout, DEFAULT_HSV_RANGES)
            if sample is not None:
                results.append(sample)
            time.sleep(SAMPLE_INTERVAL_S)

        node.destroy_node()
        rclpy.shutdown()

        print()
        if len(results) < 3:
            print(f"Only got {len(results)} usable samples — not enough to conclude.")
            print("Check the object is visible, red enough, and in range, then retry.")
            return

        rows = np.array([r["row_frac"] for r in results])
        cols = np.array([r["col_frac"] for r in results])
        diffs = np.array([abs(r["ai"] - r["ultrasonic"]) for r in results])

        print(f"=== Result ({len(results)} samples) ===")
        print(f"Ultrasonic corresponds to roughly: "
              f"{cols.mean():.1%} ± {cols.std():.1%} of width, "
              f"{rows.mean():.1%} ± {rows.std():.1%} of height")
        print(f"AI-vs-ultrasonic disagreement at that spot: "
              f"mean {diffs.mean():.2f}m, max {diffs.max():.2f}m")
        if cols.std() > 0.1 or rows.std() > 0.1:
            print("\nWarning: position varied a lot between samples — result may not be "
                  "reliable. Make sure the object stayed still and re-run.")
    finally:
        if started_camera_proc is not None:
            print("\nStopping the camera_node we started...")
            started_camera_proc.terminate()
            try:
                started_camera_proc.wait(timeout=5)
            except Exception:
                started_camera_proc.kill()


if __name__ == "__main__":
    main()
