#!/usr/bin/env python3
"""Temporarily visualize and measure the lidar mounting yaw.

This utility only rotates its own visualization. It does not publish TF or
change the running navigation stack. Put a box on the rover's physical
centreline, then adjust until that box lies on the green forward axis.
"""

import argparse
import math
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


WINDOW = "Temporary lidar yaw calibration"
IMAGE_SIZE = 700


class ScanReceiver(Node):
    """Retain the latest laser scan for the calibration display."""

    def __init__(self) -> None:
        super().__init__("lidar_yaw_calibration")
        self.scan: LaserScan | None = None
        self.lock = threading.Lock()
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, message: LaserScan) -> None:
        with self.lock:
            self.scan = message

    def latest(self) -> LaserScan | None:
        with self.lock:
            return self.scan


def render(scan: LaserScan | None, yaw_deg: float, radius_m: float) -> np.ndarray:
    image = np.zeros((IMAGE_SIZE, IMAGE_SIZE, 3), dtype=np.uint8)
    center = IMAGE_SIZE // 2
    scale = (center - 30) / radius_m

    # Physical rover forward is up; left is screen-left.
    cv2.line(image, (center, center), (center, 25), (0, 255, 0), 2)
    cv2.putText(
        image, "ROVER FORWARD", (center + 8, 42), cv2.FONT_HERSHEY_SIMPLEX,
        0.55, (0, 255, 0), 1, cv2.LINE_AA,
    )
    cv2.drawMarker(image, (center, center), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)

    if scan is not None and scan.ranges:
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
        valid = (
            np.isfinite(ranges)
            & (ranges >= scan.range_min)
            & (ranges <= min(scan.range_max, radius_m))
        )
        ranges = ranges[valid]
        angles = angles[valid] + math.radians(yaw_deg)
        # ROS +x forward, +y left -> screen up and left.
        px = center - ranges * np.sin(angles) * scale
        py = center - ranges * np.cos(angles) * scale
        image[py.astype(np.int32), px.astype(np.int32)] = (0, 220, 255)
    else:
        cv2.putText(
            image, "waiting for /scan", (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (255, 255, 255), 1, cv2.LINE_AA,
        )

    cv2.putText(
        image, f"candidate lidar yaw: {yaw_deg:+.1f} deg", (18, IMAGE_SIZE - 48),
        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA,
    )
    cv2.putText(
        image, "a: left   d: right   r: reset   q: finish", (18, IMAGE_SIZE - 18),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1, cv2.LINE_AA,
    )
    return image


def main() -> None:
    parser = argparse.ArgumentParser(description="Temporary lidar yaw calibration viewer")
    parser.add_argument("--initial", type=float, default=0.0, help="Initial yaw in degrees")
    parser.add_argument("--step", type=float, default=0.5, help="Key adjustment in degrees")
    parser.add_argument("--radius", type=float, default=2.0, help="Display radius in metres")
    args = parser.parse_args()

    rclpy.init()
    node = ScanReceiver()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    yaw_deg = args.initial

    try:
        while True:
            cv2.imshow(WINDOW, render(node.latest(), yaw_deg, args.radius))
            key = cv2.waitKey(50) & 0xFF
            if key == ord("q"):
                break
            if key == ord("a"):
                yaw_deg += args.step
            elif key == ord("d"):
                yaw_deg -= args.step
            elif key == ord("r"):
                yaw_deg = 0.0
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

    print(f"Final lidar yaw: {yaw_deg:+.1f} deg ({math.radians(yaw_deg):+.6f} rad)")
    print("This was visualization-only; copy the final radian value into nav.launch.py.")


if __name__ == "__main__":
    main()
