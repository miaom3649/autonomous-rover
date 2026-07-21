"""
Stop-and-go driving filter — runs on the Raspberry Pi.

The AI depth model that vo_node.py/orb_slam3_node rely on for scale
correction needs a round-trip HTTP call to a remote server, so it can't run
continuously at camera framerate. This node makes Nav2 drive in short
bursts instead of continuously: it passes /rover/cmd_vel_nav_raw through to
/rover/cmd_vel_nav (consumed by mode_controller_node, unchanged) for
drive_burst_s seconds, then zeroes the command and waits.

Resuming from a normal pause is event-driven only — it waits for a
/rover/odom message stamped after the pause began (i.e. a fresh,
cross-check-accepted pose has actually landed — see slam_pose_bridge.py's
accept/reject gate against ORB-SLAM3's Bundle Adjustment artifacts) before
letting the rover move again. There is deliberately no timeout fallback: if
no fix ever arrives, the rover stays put rather than driving blind. This is
a standing project rule: never resume movement without confirmation,
whatever the wait.

Also handles ORB-SLAM3 tracking loss: when SLAM loses tracking for more
than slam_loss_grace seconds, backs the rover up by backup_duration
seconds (to bring previously-mapped features back into view), then holds
still until SLAM relocalizes, then holds still a bit longer
(slam_stabilize_duration) to let SLAM accumulate fresh keyframes before
resuming normal bursts — resuming lands in PAUSED, not straight back into
DRIVING, so it still waits for a genuine fresh /rover/odom fix rather than
assuming the rover is immediately safe to move again.
"""

import enum

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32

_SLAM_OK = 2


def _stamp_to_seconds(stamp: TimeMsg) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class _State(enum.Enum):
    DRIVING = 0
    PAUSED = 1
    SLAM_BACKING = 2  # backing up after SLAM loss
    SLAM_WAITING = 3  # stopped, waiting for SLAM to relocalize
    SLAM_STABILIZE = 4  # SLAM OK but holding still to accumulate keyframes


class StopAndGoFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("stop_and_go_filter_node")

        self.declare_parameter("drive_burst_s", 2.0)
        self.declare_parameter("slam_loss_grace", 2.0)
        self.declare_parameter("backup_speed", 0.10)
        self.declare_parameter("backup_duration", 1.0)
        self.declare_parameter("slam_stabilize_duration", 5.0)

        self._burst_duration = float(self.get_parameter("drive_burst_s").value)
        self._slam_loss_grace = float(self.get_parameter("slam_loss_grace").value)
        self._backup_speed = float(self.get_parameter("backup_speed").value)
        self._backup_duration = float(self.get_parameter("backup_duration").value)
        self._slam_stabilize_duration = float(self.get_parameter("slam_stabilize_duration").value)

        self._state = _State.DRIVING
        self._state_started_at = self._now()
        self._latest_cmd = Twist()

        self._slam_state = -1
        self._slam_was_ok = False
        self._slam_lost_at: float | None = None

        self._pub_cmd = self.create_publisher(Twist, "/rover/cmd_vel_nav", 10)
        self.create_subscription(Twist, "/rover/cmd_vel_nav_raw", self._on_cmd, 10)
        self.create_subscription(Odometry, "/rover/odom", self._on_odom, 10)
        self.create_subscription(Int32, "/orb_slam3/state", self._on_slam_state, 10)
        self.create_timer(0.1, self._on_tick)

        self.get_logger().info(
            f"stop_and_go_filter_node ready — burst={self._burst_duration}s  "
            f"slam_grace={self._slam_loss_grace}s "
            f"backup={self._backup_duration}s@{self._backup_speed}m/s "
            f"stabilize={self._slam_stabilize_duration}s"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Twist) -> None:
        self._latest_cmd = msg

    def _on_odom(self, msg: Odometry) -> None:
        if self._state != _State.PAUSED:
            return
        fix_time = _stamp_to_seconds(msg.header.stamp)
        if fix_time > self._state_started_at:
            self.get_logger().info("Fresh confirmed fix received — resuming drive burst")
            self._state = _State.DRIVING
            self._state_started_at = self._now()

    def _on_slam_state(self, msg: Int32) -> None:
        prev = self._slam_state
        self._slam_state = msg.data

        if self._slam_state == _SLAM_OK:
            self._slam_was_ok = True
            self._slam_lost_at = None
            if self._state == _State.SLAM_WAITING:
                self.get_logger().info(
                    f"SLAM recovered — stabilizing {self._slam_stabilize_duration:.1f}s "
                    "before resuming navigation"
                )
                self._state = _State.SLAM_STABILIZE
                self._state_started_at = self._now()
        elif prev == _SLAM_OK and self._slam_state != _SLAM_OK:
            # First non-OK state after being OK — start the grace-period clock
            self._slam_lost_at = self._now()

    def _slam_loss_grace_expired(self) -> bool:
        if not self._slam_was_ok or self._slam_lost_at is None:
            return False
        return (self._now() - self._slam_lost_at) >= self._slam_loss_grace

    def _backup_twist(self) -> Twist:
        t = Twist()
        t.linear.x = -self._backup_speed
        return t

    def _on_tick(self) -> None:
        # SLAM recovery takes priority over normal stop-and-go bursts.
        if self._state == _State.SLAM_BACKING:
            if self._slam_state == _SLAM_OK:
                self.get_logger().info(
                    f"SLAM recovered during backup — stabilizing "
                    f"{self._slam_stabilize_duration:.1f}s"
                )
                self._state = _State.SLAM_STABILIZE
                self._state_started_at = self._now()
                self._pub_cmd.publish(Twist())
            elif (self._now() - self._state_started_at) >= self._backup_duration:
                self.get_logger().info("Backup complete — waiting for SLAM to relocalize")
                self._state = _State.SLAM_WAITING
                self._state_started_at = self._now()
                self._pub_cmd.publish(Twist())
            else:
                self._pub_cmd.publish(self._backup_twist())
            return

        if self._state == _State.SLAM_WAITING:
            # _on_slam_state transitions us out when SLAM returns to OK
            self._pub_cmd.publish(Twist())
            return

        if self._state == _State.SLAM_STABILIZE:
            if self._slam_state != _SLAM_OK:
                self.get_logger().warn("SLAM lost during stabilization — returning to wait")
                self._state = _State.SLAM_WAITING
                self._state_started_at = self._now()
            elif (self._now() - self._state_started_at) >= self._slam_stabilize_duration:
                self.get_logger().info("SLAM stable — resuming (waiting for a fresh fix)")
                self._state = _State.PAUSED
                self._state_started_at = self._now()
            self._pub_cmd.publish(Twist())
            return

        if self._slam_loss_grace_expired() and self._state not in (
            _State.SLAM_BACKING,
            _State.SLAM_WAITING,
        ):
            self.get_logger().warn(
                f"SLAM lost for >{self._slam_loss_grace:.1f}s — "
                f"backing up {self._backup_duration:.1f}s at {self._backup_speed}m/s"
            )
            self._state = _State.SLAM_BACKING
            self._state_started_at = self._now()
            self._pub_cmd.publish(self._backup_twist())
            return

        # Normal burst/pause
        if self._state == _State.DRIVING:
            if (self._now() - self._state_started_at) >= self._burst_duration:
                self._state = _State.PAUSED
                self._state_started_at = self._now()
                self._pub_cmd.publish(Twist())
                self.get_logger().info("Burst complete — pausing for a fresh fix")
            else:
                self._pub_cmd.publish(self._latest_cmd)
        elif self._state == _State.PAUSED:
            self._pub_cmd.publish(Twist())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StopAndGoFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
