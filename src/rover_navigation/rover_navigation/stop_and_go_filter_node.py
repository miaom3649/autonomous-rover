import enum

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

_RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

_SLAM_OK = 2


class _State(enum.Enum):
    IDLE = 0
    MOVING = 1
    PAUSING = 2
    SLAM_BACKING = 3   # backing up after SLAM loss
    SLAM_WAITING = 4   # stopped, waiting for SLAM to relocalize
    SLAM_STABILIZE = 5  # SLAM OK but holding still to accumulate keyframes


class StopAndGoFilterNode(Node):
    """Converts continuous Nav2 cmd_vel into stop-and-go bursts.

    Also handles SLAM tracking loss: when SLAM loses tracking for more than
    slam_loss_grace seconds, the node backs the rover up by backup_duration
    seconds (to bring previously-mapped features back into view), then stops
    until SLAM relocalizes before resuming normal navigation.
    """

    def __init__(self) -> None:
        super().__init__("stop_and_go_filter_node")

        self.declare_parameter("move_duration", 0.3)
        self.declare_parameter("pause_duration", 0.7)
        self.declare_parameter("slam_loss_grace", 2.0)
        self.declare_parameter("backup_speed", 0.10)
        self.declare_parameter("backup_duration", 1.0)
        self.declare_parameter("slam_stabilize_duration", 5.0)

        self._move_dur: float = self.get_parameter("move_duration").value
        self._pause_dur: float = self.get_parameter("pause_duration").value
        self._slam_loss_grace: float = self.get_parameter("slam_loss_grace").value
        self._backup_speed: float = self.get_parameter("backup_speed").value
        self._backup_dur: float = self.get_parameter("backup_duration").value
        self._slam_stabilize_dur: float = self.get_parameter("slam_stabilize_duration").value

        self._pub = self.create_publisher(Twist, "/rover/cmd_vel_nav_gated", _RELIABLE)
        self.create_subscription(Twist, "/rover/cmd_vel_nav", self._on_cmd, _RELIABLE)
        self.create_subscription(
            Int32, "/orb_slam3/state", self._on_slam_state,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
        )

        self._latest = Twist()
        self._state = _State.IDLE
        self._phase_start = self.get_clock().now()

        self._slam_state: int = -1
        self._slam_was_ok: bool = False
        self._slam_lost_at: rclpy.time.Time | None = None

        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"Stop-and-go filter ready — "
            f"move={self._move_dur}s pause={self._pause_dur}s  "
            f"slam_grace={self._slam_loss_grace}s "
            f"backup={self._backup_dur}s@{self._backup_speed}m/s "
            f"stabilize={self._slam_stabilize_dur}s"
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._latest = msg

    def _on_slam_state(self, msg: Int32) -> None:
        prev = self._slam_state
        self._slam_state = msg.data

        if self._slam_state == _SLAM_OK:
            self._slam_was_ok = True
            self._slam_lost_at = None
            if self._state == _State.SLAM_WAITING:
                self.get_logger().info(
                    f"SLAM recovered — stabilizing {self._slam_stabilize_dur:.1f}s "
                    "before resuming navigation"
                )
                self._state = _State.SLAM_STABILIZE
                self._phase_start = self.get_clock().now()
        elif prev == _SLAM_OK and self._slam_state != _SLAM_OK:
            # First non-OK state after being OK — start the grace-period clock
            self._slam_lost_at = self.get_clock().now()

    def _has_cmd(self) -> bool:
        return abs(self._latest.linear.x) > 0.001 or abs(self._latest.angular.z) > 0.001

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._phase_start).nanoseconds * 1e-9

    def _slam_loss_grace_expired(self) -> bool:
        if not self._slam_was_ok or self._slam_lost_at is None:
            return False
        elapsed = (self.get_clock().now() - self._slam_lost_at).nanoseconds * 1e-9
        return elapsed >= self._slam_loss_grace

    def _backup_twist(self) -> Twist:
        t = Twist()
        t.linear.x = -self._backup_speed
        return t

    def _tick(self) -> None:
        # SLAM recovery takes priority over normal stop-and-go
        if self._state == _State.SLAM_BACKING:
            if self._slam_state == _SLAM_OK:
                self.get_logger().info(
                    f"SLAM recovered during backup — stabilizing {self._slam_stabilize_dur:.1f}s"
                )
                self._state = _State.SLAM_STABILIZE
                self._phase_start = self.get_clock().now()
                self._pub.publish(Twist())
            elif self._elapsed() >= self._backup_dur:
                self.get_logger().info("Backup complete — waiting for SLAM to relocalize")
                self._state = _State.SLAM_WAITING
                self._phase_start = self.get_clock().now()
                self._pub.publish(Twist())
            else:
                self._pub.publish(self._backup_twist())
            return

        if self._state == _State.SLAM_WAITING:
            # _on_slam_state transitions us out when SLAM returns to OK
            self._pub.publish(Twist())
            return

        if self._state == _State.SLAM_STABILIZE:
            # Hold still while SLAM builds keyframes; resume after stabilize period
            if self._slam_state != _SLAM_OK:
                # Lost tracking again during stabilize — back to waiting
                self.get_logger().warn("SLAM lost during stabilization — returning to wait")
                self._state = _State.SLAM_WAITING
                self._phase_start = self.get_clock().now()
            elif self._elapsed() >= self._slam_stabilize_dur:
                self.get_logger().info("SLAM stable — resuming navigation")
                self._state = _State.IDLE
                self._phase_start = self.get_clock().now()
            self._pub.publish(Twist())
            return

        # Check for SLAM loss before normal operation
        if self._slam_loss_grace_expired() and self._state not in (
            _State.SLAM_BACKING, _State.SLAM_WAITING
        ):
            self.get_logger().warn(
                f"SLAM lost for >{self._slam_loss_grace:.1f}s — "
                f"backing up {self._backup_dur:.1f}s at {self._backup_speed}m/s"
            )
            self._state = _State.SLAM_BACKING
            self._phase_start = self.get_clock().now()
            self._pub.publish(self._backup_twist())
            return

        # Normal stop-and-go
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
        node.destroy_node()
        rclpy.shutdown()
