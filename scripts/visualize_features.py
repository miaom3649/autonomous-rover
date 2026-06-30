#!/usr/bin/env python3
"""
Visualize ORB feature detection from the camera topic.

Usage:
    python3 scripts/visualize_features.py           # save to log/features.jpg (1 Hz)
    python3 scripts/visualize_features.py --display  # cv2.imshow via X11 (ssh -X)
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

_LOG_DIR = os.path.expanduser("~/dev/autonomous-rover/log")


class FeatureVisualizer(Node):
    def __init__(self, display: bool) -> None:
        super().__init__("feature_visualizer")
        self._display = display
        self._orb = cv2.ORB_create(
            nfeatures=1000,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=31,
            fastThreshold=3,
        )
        self._sub = self.create_subscription(
            Image,
            "/rover/camera/image_raw",
            self._on_image,
            qos_profile_sensor_data,
        )
        os.makedirs(_LOG_DIR, exist_ok=True)
        self._last_save = 0.0
        self.get_logger().info("Waiting for /rover/camera/image_raw ...")

    def _on_image(self, msg: Image) -> None:
        frame = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        keypoints = self._orb.detect(gray, None)

        annotated = cv2.drawKeypoints(
            frame,
            keypoints,
            None,
            color=(0, 255, 0),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )
        cv2.putText(
            annotated,
            f"ORB features: {len(keypoints)}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )
        bgr = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)

        if self._display:
            cv2.imshow("ORB Features", bgr)
            cv2.waitKey(1)
        else:
            now = time.time()
            if now - self._last_save >= 1.0:
                path = os.path.join(_LOG_DIR, "features.jpg")
                cv2.imwrite(path, bgr)
                self._last_save = now
                self.get_logger().info(f"{len(keypoints)} features — saved {path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--display", action="store_true")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = FeatureVisualizer(display=args.display)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
