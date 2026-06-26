#!/usr/bin/env python3
"""Print rover position (x, y, yaw) from /rover/odom at 1 Hz."""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class ShowPose(Node):
    def __init__(self) -> None:
        super().__init__("show_pose")
        self._latest: Odometry | None = None
        self.create_subscription(Odometry, "/rover/odom", self._cb, 10)
        self.create_timer(1.0, self._print)

    def _cb(self, msg: Odometry) -> None:
        self._latest = msg

    def _print(self) -> None:
        if self._latest is None:
            print("waiting for /rover/odom…")
            return
        p = self._latest.pose.pose.position
        q = self._latest.pose.pose.orientation
        yaw = math.degrees(math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        ))
        print(f"x={p.x:+.3f}  y={p.y:+.3f}  yaw={yaw:+.1f}°")


def main() -> None:
    rclpy.init()
    node = ShowPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
