import enum

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist

_RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class _State(enum.Enum):
    IDLE = 0
    MOVING = 1
    PAUSING = 2


class StopAndGoFilterNode(Node):
    """Converts continuous Nav2 cmd_vel into stop-and-go bursts.

    Subscribes to /rover/cmd_vel_nav and publishes /rover/cmd_vel_nav_gated.
    While a non-zero command is received it alternates between forwarding the
    command for move_duration seconds and publishing zero for pause_duration
    seconds, giving the camera time to capture a blur-free frame for SLAM.
    """

    def __init__(self) -> None:
        super().__init__("stop_and_go_filter_node")

        self.declare_parameter("move_duration", 0.3)
        self.declare_parameter("pause_duration", 0.7)

        self._move_dur: float = self.get_parameter("move_duration").value
        self._pause_dur: float = self.get_parameter("pause_duration").value

        self._pub = self.create_publisher(Twist, "/rover/cmd_vel_nav_gated", _RELIABLE)
        self.create_subscription(Twist, "/rover/cmd_vel_nav", self._on_cmd, _RELIABLE)

        self._latest = Twist()
        self._state = _State.IDLE
        self._phase_start = self.get_clock().now()

        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"Stop-and-go filter ready — move={self._move_dur}s pause={self._pause_dur}s"
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._latest = msg

    def _has_cmd(self) -> bool:
        return abs(self._latest.linear.x) > 0.001 or abs(self._latest.angular.z) > 0.001

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._phase_start).nanoseconds * 1e-9

    def _tick(self) -> None:
        if self._state == _State.IDLE:
            if self._has_cmd():
                self._state = _State.MOVING
                self._phase_start = self.get_clock().now()
                self._pub.publish(self._latest)
            else:
                self._pub.publish(Twist())

        elif self._state == _State.MOVING:
            if not self._has_cmd():
                self._state = _State.IDLE
                self._pub.publish(Twist())
            elif self._elapsed() >= self._move_dur:
                self._state = _State.PAUSING
                self._phase_start = self.get_clock().now()
                self.get_logger().debug("→ pause")
                self._pub.publish(Twist())
            else:
                self._pub.publish(self._latest)

        elif self._state == _State.PAUSING:
            if not self._has_cmd():
                self._state = _State.IDLE
                self._pub.publish(Twist())
            elif self._elapsed() >= self._pause_dur:
                self._state = _State.MOVING
                self._phase_start = self.get_clock().now()
                self.get_logger().debug("→ move")
                self._pub.publish(self._latest)
            else:
                self._pub.publish(Twist())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StopAndGoFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()
