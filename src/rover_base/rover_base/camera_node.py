"""Pi Camera publisher for the rover's perception and diagnostic nodes."""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraNode(Node):
    """Publish RGB frames from Picamera2 on ``/rover/camera/image_raw``."""

    def __init__(self) -> None:
        super().__init__("camera_node")
        self.declare_parameter("use_sim", False)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 10.0)

        self._use_sim = bool(self.get_parameter("use_sim").value)
        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)
        if self._width <= 0 or self._height <= 0 or fps <= 0.0:
            raise ValueError("camera width, height, and fps must be positive")

        self._publisher = self.create_publisher(
            Image, "/rover/camera/image_raw", qos_profile_sensor_data
        )
        self._camera = None
        if not self._use_sim:
            from picamera2 import Picamera2  # type: ignore[import-not-found]

            self._camera = Picamera2()
            configuration = self._camera.create_preview_configuration(
                main={"size": (self._width, self._height), "format": "RGB888"},
                controls={"FrameRate": fps},
            )
            self._camera.configure(configuration)
            self._camera.start()

        self.create_timer(1.0 / fps, self._publish_frame)
        self.get_logger().info(
            f"Camera ready: {self._width}x{self._height} @ {fps:.1f} Hz (sim={self._use_sim})"
        )

    def _publish_frame(self) -> None:
        if self._camera is None:
            frame = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        else:
            frame = np.ascontiguousarray(self._camera.capture_array("main"))
        if frame.shape != (self._height, self._width, 3):
            self.get_logger().warn(
                f"Unexpected camera frame shape: {frame.shape}", throttle_duration_sec=5.0
            )
            return

        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "camera_optical_frame"
        message.height = self._height
        message.width = self._width
        message.encoding = "rgb8"
        message.is_bigendian = False
        message.step = self._width * 3
        message.data = frame.tobytes()
        self._publisher.publish(message)

    def destroy_node(self) -> None:
        if self._camera is not None:
            try:
                self._camera.stop()
                self._camera.close()
            except Exception:
                pass
            self._camera = None
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
