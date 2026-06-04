#!/usr/bin/env python3
# Hardware diagnostic script — direct SDK access intentional.
# Not a ROS node. Run on physical hardware only.
"""
Capture calibration frames from /rover/camera/image_raw.

A live preview window shows the camera feed with detected checkerboard corners
drawn in real time. Frames are saved automatically whenever a checkerboard is
detected, subject to a cooldown between captures.

Press 'q' to quit.

Usage:
    python3 scripts/capture_calibration_frames.py [--size COLSxROWS] [--cooldown SECONDS]
"""

import argparse
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


OUTPUT_DIR = Path("/tmp/calibration_frames")
MIN_FRAMES = 20
DEFAULT_BOARD_SIZE = (8, 6)
DEFAULT_COOLDOWN_S = 2.0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--size", default="8x6",
                   help="Inner corner count as COLSxROWS (default: 8x6)")
    p.add_argument("--cooldown", type=float, default=DEFAULT_COOLDOWN_S,
                   help="Seconds between auto-captures (default: 2.0)")
    return p.parse_args()


class _FrameCapture(Node):
    def __init__(self) -> None:
        super().__init__("frame_capture")
        self._latest: np.ndarray | None = None
        self._lock = threading.Lock()

        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        self.create_subscription(
            Image, "/rover/camera/image_raw", self._callback, qos_profile_sensor_data
        )

    def _callback(self, msg: Image) -> None:
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )
        with self._lock:
            self._latest = arr

    def latest_bgr(self) -> np.ndarray | None:
        with self._lock:
            frame = self._latest
        if frame is None:
            return None
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


def main() -> None:
    args = parse_args()
    cols, rows = (int(x) for x in args.size.lower().split("x"))
    board_size = (cols, rows)

    rclpy.init()
    node = _FrameCapture()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f"Saving frames to: {OUTPUT_DIR}")
    print(f"Board: {cols}x{rows} inner corners")
    print(f"Auto-capture cooldown: {args.cooldown}s")
    print(f"Recommended minimum: {MIN_FRAMES} frames from different angles")
    print("q — quit\n")

    cv2.namedWindow("Calibration Preview", cv2.WINDOW_NORMAL)

    count = 0
    last_capture_time = 0.0

    try:
        while True:
            bgr = node.latest_bgr()

            if bgr is None:
                placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(
                    placeholder, "Waiting for camera...", (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1,
                )
                cv2.imshow("Calibration Preview", placeholder)
            else:
                display = bgr.copy()
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                found, corners = cv2.findChessboardCorners(gray, board_size, None)

                now = time.monotonic()
                cooldown_remaining = max(0.0, args.cooldown - (now - last_capture_time))

                if found:
                    corners_refined = cv2.cornerSubPix(
                        gray, corners, (11, 11), (-1, -1),
                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
                    )
                    cv2.drawChessboardCorners(display, board_size, corners_refined, found)

                    if cooldown_remaining == 0.0:
                        path = OUTPUT_DIR / f"frame_{count:03d}.jpg"
                        cv2.imwrite(str(path), bgr)
                        count += 1
                        last_capture_time = now
                        remaining = max(0, MIN_FRAMES - count)
                        print(f"Saved {path}  ({count} captured, {remaining} more recommended)")

                # Status overlay
                if found and cooldown_remaining > 0.0:
                    status = f"Detected — next in {cooldown_remaining:.1f}s"
                    color = (0, 200, 255)
                elif found:
                    status = "Detected — saving!"
                    color = (0, 255, 0)
                else:
                    status = "No checkerboard detected"
                    color = (0, 0, 220)

                cv2.putText(display, f"Captured: {count}  |  {status}",
                            (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
                cv2.imshow("Calibration Preview", display)

            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

    print(f"\nDone. {count} frames saved to {OUTPUT_DIR}")
    if count < MIN_FRAMES:
        print(f"[!] Only {count} frames — recommend at least {MIN_FRAMES} for accurate calibration.")
    else:
        print("Run next step:  python3 scripts/run_camera_calibration.py")


if __name__ == "__main__":
    main()
