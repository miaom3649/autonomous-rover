#!/usr/bin/env python3
"""Send a Nav2 goal N metres ahead of the rover's current SLAM position."""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavForward(Node):
    def __init__(self, distance: float) -> None:
        super().__init__("nav_forward")
        self._distance = distance
        self._odom: Odometry | None = None
        self._sub = self.create_subscription(Odometry, "/rover/odom", self._on_odom, 10)
        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def run(self) -> bool:
        self.get_logger().info("Waiting for odom…")
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=5)
        while self._odom is None and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._odom is None:
            self.get_logger().error("No /rover/odom received — is SLAM running?")
            return False

        x = self._odom.pose.pose.position.x
        y = self._odom.pose.pose.position.y
        q = self._odom.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))

        gx = x + self._distance * math.cos(yaw)
        gy = y + self._distance * math.sin(yaw)
        self.get_logger().info(
            f"Current ({x:.3f}, {y:.3f}) yaw={math.degrees(yaw):.1f}° → "
            f"goal ({gx:.3f}, {gy:.3f})"
        )

        self.get_logger().info("Waiting for Nav2 action server…")
        self._client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.w = 1.0

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False

        self.get_logger().info("Goal accepted — navigating…")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("Done")
        return True


def main() -> None:
    distance = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    rclpy.init()
    node = NavForward(distance)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
