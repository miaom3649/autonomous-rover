"""
Depth bridge — runs on the Raspberry Pi.

Monitors /rover/cmd_vel for stop events, grabs one frame per stop,
POSTs it to the Windows depth server, and publishes the metric depth
map as /rover/camera/depth (32FC1, meters) with the source frame's
original timestamp so ORB-SLAM3 RGBD sync works correctly.
"""
import io
import threading
import time
from typing import Optional

import cv2
import numpy as np
import requests
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_bridge_node")

        self.declare_parameter("depth_server_url", "http://192.168.1.100:8765/depth")
        self.declare_parameter("settle_delay", 0.5)
        self.declare_parameter("request_timeout", 0.8)

        self._server_url: str = self.get_parameter("depth_server_url").value
        self._settle_delay: float = float(self.get_parameter("settle_delay").value)
        self._timeout: float = float(self.get_parameter("request_timeout").value)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame: Optional[Image] = None
        self._last_moving_time: float = time.monotonic()
        self._processing: bool = False

        self._sub_cmd = self.create_subscription(
            Twist, "/rover/cmd_vel", self._on_cmd_vel, 10
        )
        self._sub_img = self.create_subscription(
            Image, "/rover/camera/image_raw", self._on_image, qos_profile_sensor_data
        )
        self._pub_depth = self.create_publisher(
            Image, "/rover/camera/depth", qos_profile_sensor_data
        )

        self.create_timer(0.1, self._check_stop)

        self.get_logger().info(f"depth_bridge_node ready  server={self._server_url}")

    def _on_cmd_vel(self, msg: Twist) -> None:
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            with self._lock:
                self._last_moving_time = time.monotonic()

    def _on_image(self, msg: Image) -> None:
        with self._lock:
            self._latest_frame = msg

    def _check_stop(self) -> None:
        with self._lock:
            elapsed = time.monotonic() - self._last_moving_time
            frame = self._latest_frame
            if elapsed < self._settle_delay or frame is None or self._processing:
                return
            self._processing = True
            self._latest_frame = None  # consume — prevents double-firing this stop cycle

        t = threading.Thread(target=self._fetch_and_publish, args=(frame,), daemon=True)
        t.start()

    def _fetch_and_publish(self, frame_msg: Image) -> None:
        try:
            rgb = self._bridge.imgmsg_to_cv2(frame_msg, "rgb8")
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            _, enc = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])

            resp = requests.post(
                self._server_url,
                data=bytes(enc),
                headers={"Content-Type": "image/jpeg"},
                timeout=self._timeout,
            )
            resp.raise_for_status()

            h = int(resp.headers["X-Depth-Height"])
            w = int(resp.headers["X-Depth-Width"])
            depth = np.frombuffer(resp.content, dtype=np.float32).reshape(h, w)

            depth_msg = self._bridge.cv2_to_imgmsg(depth, encoding="32FC1")
            depth_msg.header = frame_msg.header
            self._pub_depth.publish(depth_msg)

            self.get_logger().debug(
                f"depth {w}x{h}  min={depth.min():.2f}m  max={depth.max():.2f}m"
            )
        except requests.Timeout:
            self.get_logger().warn("depth server timeout — skipping this stop cycle")
        except Exception as exc:
            self.get_logger().error(f"depth fetch failed: {exc}")
        finally:
            with self._lock:
                self._processing = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
