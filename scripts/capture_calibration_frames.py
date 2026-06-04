#!/usr/bin/env python3
# Hardware diagnostic script — direct SDK access intentional.
# Not a ROS node. Run on physical hardware only.
"""
Capture calibration frames from /rover/camera/image_raw.

A live preview window shows the camera feed.
Press SPACE to save the current frame, 'q' to quit.
Saved frames go to OUTPUT_DIR as frame_000.jpg, frame_001.jpg, ...

Usage:
    python3 scripts/capture_calibration_frames.py
"""

import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


OUTPUT_DIR = Path("/tmp/calibration_frames")
MIN_FRAMES = 20


class _FrameCapture(Node):
    def __init__(self) -> None:
        super().__init__("frame_capture")
        self._latest: np.ndarray | None = None
        self._lock = threading.Lock()
        self._count = 0

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

    def save_frame(self, bgr: np.ndarray) -> None:
        path = OUTPUT_DIR / f"frame_{self._count:03d}.jpg"
        cv2.imwrite(str(path), bgr)
        self._count += 1
        remaining = max(0, MIN_FRAMES - self._count)
        print(f"Saved {path}  ({self._count} captured, {remaining} more recommended)")


def main() -> None:
    rclpy.init()
    node = _FrameCapture()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f"Saving frames to: {OUTPUT_DIR}")
    print(f"Recommended minimum: {MIN_FRAMES} frames from different angles")
    print("SPACE — capture frame   q — quit\n")

    cv2.namedWindow("Calibration Preview", cv2.WINDOW_NORMAL)

    try:
        while True:
            bgr = node.latest_bgr()

            if bgr is None:
                # Show a blank placeholder until the first frame arrives
                placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(
                    placeholder, "Waiting for camera...", (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1,
                )
                cv2.imshow("Calibration Preview", placeholder)
            else:
                overlay = bgr.copy()
                cv2.putText(
                    overlay,
                    f"Captured: {node._count}  |  SPACE=save  q=quit",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1,
                )
                cv2.imshow("Calibration Preview", overlay)

            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                break
            elif key == ord(" "):
                if bgr is None:
                    print("[!] No frame received yet — is camera_node active?")
                else:
                    node.save_frame(bgr)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

    print(f"\nDone. {node._count} frames saved to {OUTPUT_DIR}")
    if node._count < MIN_FRAMES:
        print(f"[!] Only {node._count} frames — recommend at least {MIN_FRAMES} for accurate calibration.")
    else:
        print("Run next step:  python3 scripts/run_camera_calibration.py")


if __name__ == "__main__":
    main()
