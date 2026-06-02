import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


_DEFAULT_WIDTH = 320
_DEFAULT_HEIGHT = 240
_DEFAULT_RATE_HZ = 15.0
_DEFAULT_FRAME_ID = "camera_link"

# Checkerboard square size (pixels) used in sim mode.
_SIM_SQUARE_PX = 40


class CameraNode(LifecycleNode):
    """Lifecycle driver node for the Pi Camera (CSI, via picamera2).

    Publishes:
        /rover/camera/image_raw   (sensor_msgs/Image)       — raw RGB frames
        /rover/camera/camera_info (sensor_msgs/CameraInfo)  — intrinsics (zeros until calibrated)

    Parameters:
        use_sim        (bool,  default False)  — publish synthetic checkerboard frames
        publish_rate_hz (float, default 15.0)  — capture rate
        frame_width    (int,   default 640)
        frame_height   (int,   default 480)
        frame_id       (str,   default "camera_link")
    """

    def __init__(self) -> None:
        super().__init__("camera_node")
        self._timer = None
        self._camera = None
        self._sim_frame: np.ndarray | None = None

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
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

        if self._use_sim:
            self._sim_frame = self._make_checkerboard()
        else:
            if not self._init_hardware():
                return TransitionCallbackReturn.FAILURE

        self.get_logger().info(
            f"Configured — mode={'sim' if self._use_sim else 'hardware'}, "
            f"resolution={self._width}x{self._height}, rate={self._rate_hz} Hz"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        if not self._use_sim and self._camera is not None:
            self._camera.start()
        self._timer = self.create_timer(1.0 / self._rate_hz, self._publish_frame)
        self.get_logger().info("Activated — publishing camera frames.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        if not self._use_sim and self._camera is not None:
            self._camera.stop()
        self.get_logger().info("Deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self._camera is not None:
            self._camera.close()
            self._camera = None
        self._sim_frame = None
        self.get_logger().info("Cleaned up.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self._camera is not None:
            self._camera.close()
            self._camera = None
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _init_hardware(self) -> bool:
        try:
            from picamera2 import Picamera2  # type: ignore
        except ImportError as exc:
            msg = (
                "picamera2 libcamera bindings not found — see README.md."
                if "libcamera" in str(exc)
                else "picamera2 not installed. Use use_sim:=true on the dev machine."
            )
            self.get_logger().error(msg)
            return False
        try:
            self._camera = Picamera2()
            cfg = self._camera.create_video_configuration(
                main={"format": "RGB888", "size": (self._width, self._height)}
            )
            self._camera.configure(cfg)
        except Exception as exc:
            self.get_logger().error(f"Failed to initialise Pi Camera: {exc}")
            return False
        return True

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
        try:
            return self._camera.capture_array()
        except Exception as exc:
            self.get_logger().warning(f"Frame capture failed: {exc}")
            return None

    def _capture_sim(self) -> np.ndarray:
        return self._sim_frame

    def _make_checkerboard(self) -> np.ndarray:
        """Generate a static grey/white checkerboard for sim mode."""
        xs = np.arange(self._width)
        ys = np.arange(self._height)
        xx, yy = np.meshgrid(xs, ys)
        mask = ((xx // _SIM_SQUARE_PX) + (yy // _SIM_SQUARE_PX)) % 2 == 0
        frame = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        frame[mask] = [200, 200, 200]
        return frame


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
