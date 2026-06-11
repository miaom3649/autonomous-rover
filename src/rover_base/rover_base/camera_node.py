import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


_DEFAULT_WIDTH = 320
_DEFAULT_HEIGHT = 240
_DEFAULT_RATE_HZ = 15.0
_DEFAULT_FRAME_ID = "camera_link"

_SIM_SQUARE_PX = 40


class CameraNode(Node):
    """Camera driver node for the Pi Camera (CSI, via picamera2).

    Publishes:
        /rover/camera/image_raw   (sensor_msgs/Image)
        /rover/camera/camera_info (sensor_msgs/CameraInfo)
    """

    def __init__(self) -> None:
        super().__init__("camera_node")

        self.declare_parameter("use_sim", False)
        self.declare_parameter("publish_rate_hz", _DEFAULT_RATE_HZ)
        self.declare_parameter("frame_width", _DEFAULT_WIDTH)
        self.declare_parameter("frame_height", _DEFAULT_HEIGHT)
        self.declare_parameter("frame_id", _DEFAULT_FRAME_ID)

        self._use_sim: bool = self.get_parameter("use_sim").value
        self._rate_hz: float = self.get_parameter("publish_rate_hz").value
        self._width: int = self.get_parameter("frame_width").value
        self._height: int = self.get_parameter("frame_height").value
        self._frame_id: str = self.get_parameter("frame_id").value

        self._pub_image = self.create_publisher(
            Image, "/rover/camera/image_raw", qos_profile_sensor_data
        )
        self._pub_info = self.create_publisher(
            CameraInfo, "/rover/camera/camera_info", qos_profile_sensor_data
        )

        self._camera = None
        self._sim_frame: np.ndarray | None = None

        if self._use_sim:
            self._sim_frame = self._make_checkerboard()
            self.get_logger().info(
                f"Camera node ready (sim), {self._width}x{self._height} @ {self._rate_hz} Hz"
            )
        else:
            self._init_hardware()

        self.create_timer(1.0 / self._rate_hz, self._publish_frame)

    def _init_hardware(self) -> None:
        try:
            from picamera2 import Picamera2  # type: ignore
        except ImportError as exc:
            self.get_logger().error(f"picamera2 not available: {exc}")
            return
        try:
            self._camera = Picamera2()
            cfg = self._camera.create_video_configuration(
                main={"format": "RGB888", "size": (self._width, self._height)}
            )
            self._camera.configure(cfg)
            self._camera.start()
            self.get_logger().info(
                f"Camera ready — {self._width}x{self._height} @ {self._rate_hz} Hz"
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to initialise Pi Camera: {exc}")

    def _publish_frame(self) -> None:
        frame = self._capture_sim() if self._use_sim else self._capture_hardware()
        if frame is None:
            return

        stamp = self.get_clock().now().to_msg()

        img_msg = Image()
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self._frame_id
        img_msg.height = self._height
        img_msg.width = self._width
        img_msg.encoding = "rgb8"
        img_msg.is_bigendian = False
        img_msg.step = self._width * 3
        img_msg.data = frame.tobytes()
        self._pub_image.publish(img_msg)

        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self._frame_id
        info_msg.width = self._width
        info_msg.height = self._height
        self._pub_info.publish(info_msg)

    def _capture_hardware(self) -> np.ndarray | None:
        if self._camera is None:
            return None
        try:
            return self._camera.capture_array()
        except Exception as exc:
            self.get_logger().warning(f"Frame capture failed: {exc}")
            return None

    def _capture_sim(self) -> np.ndarray:
        return self._sim_frame

    def _make_checkerboard(self) -> np.ndarray:
        xs = np.arange(self._width)
        ys = np.arange(self._height)
        xx, yy = np.meshgrid(xs, ys)
        mask = ((xx // _SIM_SQUARE_PX) + (yy // _SIM_SQUARE_PX)) % 2 == 0
        frame = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        frame[mask] = [200, 200, 200]
        return frame

    def destroy_node(self) -> None:
        if self._camera is not None:
            self._camera.stop()
            self._camera.close()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
