"""
Stop-and-go driving filter — runs on the Raspberry Pi.

The AI depth model that vo_node.py relies on for scale correction needs a
round-trip HTTP call to a remote server, so it can't run continuously at
camera framerate. This node makes Nav2 drive in short bursts instead of
continuously: it passes /rover/cmd_vel_nav_raw through to /rover/cmd_vel_nav
(consumed by mode_controller_node, unchanged) for drive_burst_s seconds,
then zeroes the command and waits.

Resuming is event-driven only — it waits for a /rover/odom message stamped
after the pause began (i.e. a fresh vo_node fix has actually landed) before
letting the rover move again. There is deliberately no timeout fallback: if
no fix ever arrives, the rover stays put rather than driving blind. This
mirrors the project's standing rule from the ORB-SLAM3 era's stop-and-go
logic — never resume movement without confirmation, whatever the wait.
"""

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _stamp_to_seconds(stamp: TimeMsg) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class StopAndGoFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("stop_and_go_filter_node")

        self.declare_parameter("drive_burst_s", 2.0)

        self._burst_duration = float(self.get_parameter("drive_burst_s").value)
        self._driving = True
        self._state_started_at = self._now()

        self._pub_cmd = self.create_publisher(Twist, "/rover/cmd_vel_nav", 10)
        self.create_subscription(Twist, "/rover/cmd_vel_nav_raw", self._on_cmd, 10)
        self.create_subscription(Odometry, "/rover/odom", self._on_odom, 10)
        self.create_timer(0.1, self._on_tick)

        self.get_logger().info(f"stop_and_go_filter_node ready — burst={self._burst_duration}s")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Twist) -> None:
        if self._driving:
            self._pub_cmd.publish(msg)

    def _on_odom(self, msg: Odometry) -> None:
        if self._driving:
            return
        fix_time = _stamp_to_seconds(msg.header.stamp)
        if fix_time > self._state_started_at:
            self.get_logger().info("Fresh VO fix received — resuming drive burst")
            self._driving = True
            self._state_started_at = self._now()

    def _on_tick(self) -> None:
        if self._driving and (self._now() - self._state_started_at) >= self._burst_duration:
            self._driving = False
            self._state_started_at = self._now()
            self._pub_cmd.publish(Twist())
            self.get_logger().info("Burst complete — pausing for a fresh VO fix")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StopAndGoFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
