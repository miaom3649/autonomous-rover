"""Send rover camera frames to the Windows YOLO service."""

import json
import threading
import time

import cv2
import numpy as np
import requests
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ObjectDetectorNode(Node):
    """Bridge camera frames to HTTP inference and publish normalized boxes."""

    def __init__(self) -> None:
        super().__init__("object_detector_node")
        self.declare_parameter("server_url", "")
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("timeout_s", 3.0)
        self.declare_parameter(
            "ground_labels", ["person", "chair", "couch", "potted plant", "suitcase"]
        )
        self._url = str(self.get_parameter("server_url").value)
        self._period = 1.0 / float(self.get_parameter("rate_hz").value)
        self._timeout = float(self.get_parameter("timeout_s").value)
        self._ground_labels = set(self.get_parameter("ground_labels").value)
        self._session = requests.Session()
        # The Pi may use an internet proxy, but the detector is a direct LAN
        # service. Never route camera frames through that proxy.
        self._session.trust_env = False
        self._last_request = 0.0
        self._busy = False
        self._lock = threading.Lock()
        self._publisher = self.create_publisher(String, "/rover/object_detections", 10)
        self.create_subscription(
            Image, "/rover/camera/image_raw", self._on_image, qos_profile_sensor_data
        )
        if self._url:
            self.get_logger().info(f"Object detector ready: {self._url}")
        else:
            self.get_logger().warn("Object detector disabled: server_url is empty")

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        with self._lock:
            if not self._url or self._busy or now - self._last_request < self._period:
                return
            self._busy = True
            self._last_request = now
        try:
            rows = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(
                message.height, message.step
            )
            frame = rows[:, : message.width * 3].reshape(message.height, message.width, 3)
            if message.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif message.encoding != "bgr8":
                raise ValueError(f"unsupported encoding: {message.encoding}")
            stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
                message.header.stamp.nanosec
            )
            threading.Thread(
                target=self._request, args=(frame.copy(), stamp_ns), daemon=True
            ).start()
        except Exception as exc:
            with self._lock:
                self._busy = False
            self.get_logger().warn(f"Could not prepare detector frame: {exc}")

    def _request(self, frame: np.ndarray, stamp_ns: int) -> None:
        try:
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise RuntimeError("JPEG encoding failed")
            response = self._session.post(
                self._url,
                data=encoded.tobytes(),
                headers={"Content-Type": "image/jpeg"},
                timeout=self._timeout,
            )
            response.raise_for_status()
            payload = response.json()
            detections = payload["detections"]
            for detection in detections:
                detection["project_to_ground"] = detection["label"] in self._ground_labels
                detection["image_stamp_ns"] = stamp_ns
            self._publisher.publish(String(data=json.dumps(detections)))
        except Exception as exc:
            self.get_logger().warn(f"Object detection request failed: {exc}")
        finally:
            with self._lock:
                self._busy = False


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
