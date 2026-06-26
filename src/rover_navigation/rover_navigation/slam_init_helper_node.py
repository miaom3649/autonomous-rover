import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

_SLAM_OK = 2


class SlamInitHelperNode(Node):
    """Sweep the camera pan servo left-right whenever ORB-SLAM3 is not tracking.

    Monocular SLAM needs parallax between frames to triangulate depth.
    Sweeps during initial startup AND after tracking loss re-initialization.
    Stops and returns to home while SLAM is actively tracking (state == OK)
    so navigation position estimates are not corrupted.
    """

    def __init__(self) -> None:
        super().__init__("slam_init_helper_node")

        self.declare_parameter("pan_home", 16.0)
        self.declare_parameter("pan_amplitude", 5.0)
        self.declare_parameter("sweep_period", 4.0)

        self._pan_home = float(self.get_parameter("pan_home").value)
        self._amplitude = float(self.get_parameter("pan_amplitude").value)
        self._period = float(self.get_parameter("sweep_period").value)

        self._tracking = False
        self._sweep_start_ns = self.get_clock().now().nanoseconds

        self._state_sub = self.create_subscription(
            Int32, "/orb_slam3/state", self._on_state, 10
        )
        self._pan_pub = self.create_publisher(Float32, "/rover/camera/pan", 10)

        self.create_timer(0.1, self._sweep)

        self.get_logger().info(
            f"SLAM init helper: sweeping pan ±{self._amplitude:.0f}° "
            f"around {self._pan_home:.0f}° during non-tracking states"
        )

    def _on_state(self, msg: Int32) -> None:
        tracking_now = msg.data == _SLAM_OK
        if tracking_now and not self._tracking:
            self._tracking = True
            out = Float32()
            out.data = self._pan_home
            self._pan_pub.publish(out)
            self.get_logger().info("SLAM tracking OK — camera pan returned to home")
        elif not tracking_now and self._tracking:
            self._tracking = False
            self._sweep_start_ns = self.get_clock().now().nanoseconds
            self.get_logger().info("SLAM lost — resuming pan sweep")

    def _sweep(self) -> None:
        if self._tracking:
            return
        t = (self.get_clock().now().nanoseconds - self._sweep_start_ns) * 1e-9
        angle = self._pan_home + self._amplitude * math.sin(2 * math.pi * t / self._period)
        out = Float32()
        out.data = float(angle)
        self._pan_pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SlamInitHelperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
