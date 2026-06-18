import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


_MANUAL = "MANUAL"
_AUTO = "AUTO"

_RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class ModeControllerNode(Node):
    """Arbitrates cmd_vel between MANUAL (teleop) and AUTO (Nav2) modes.

    Topics subscribed:
        /rover/mode        (std_msgs/String)  — "MANUAL" or "AUTO"
        /rover/estop       (std_msgs/Bool)    — True triggers emergency stop
        /rover/cmd_vel_teleop  (geometry_msgs/Twist) — teleop commands
        /rover/cmd_vel_nav     (geometry_msgs/Twist) — Nav2 commands

    Topics published:
        /rover/cmd_vel     (geometry_msgs/Twist) — forwarded to drive_node
        /rover/current_mode (std_msgs/String)    — current active mode
    """

    def __init__(self) -> None:
        super().__init__("mode_controller_node")

        self._mode: str = _MANUAL
        self._estopped: bool = False

        self._pub_cmd = self.create_publisher(Twist, "/rover/cmd_vel", _RELIABLE)
        self._pub_mode = self.create_publisher(String, "/rover/current_mode", _RELIABLE)

        self.create_subscription(String, "/rover/mode", self._on_mode, _RELIABLE)
        self.create_subscription(Bool, "/rover/estop", self._on_estop, _RELIABLE)
        self.create_subscription(
            Twist, "/rover/cmd_vel_teleop", self._on_teleop, _RELIABLE
        )
        self.create_subscription(
            Twist, "/rover/cmd_vel_nav", self._on_nav, _RELIABLE
        )

        self.get_logger().info(f"Mode controller ready — starting in {self._mode} mode")
        self._publish_mode()

    def _on_mode(self, msg: String) -> None:
        requested = msg.data.upper()
        if requested not in (_MANUAL, _AUTO):
            self.get_logger().warn(f"Unknown mode requested: {msg.data!r} — ignoring")
            return
        if requested == self._mode:
            return
        self._mode = requested
        self.get_logger().info(f"Mode → {self._mode}")
        self._publish_mode()
        if self._mode == _MANUAL:
            self._send_zero()

    def _on_estop(self, msg: Bool) -> None:
        if msg.data and not self._estopped:
            self._estopped = True
            self.get_logger().warn("EMERGENCY STOP activated")
            self._send_zero()
        elif not msg.data and self._estopped:
            self._estopped = False
            self.get_logger().info("Emergency stop cleared — mode: {self._mode}")

    def _on_teleop(self, msg: Twist) -> None:
        if self._estopped or self._mode != _MANUAL:
            return
        self._pub_cmd.publish(msg)

    def _on_nav(self, msg: Twist) -> None:
        if self._estopped or self._mode != _AUTO:
            return
        self._pub_cmd.publish(msg)

    def _send_zero(self) -> None:
        self._pub_cmd.publish(Twist())

    def _publish_mode(self) -> None:
        msg = String()
        msg.data = self._mode
        self._pub_mode.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ModeControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
