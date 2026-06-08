import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range


class UltrasonicToScanNode(Node):
    """Convert /rover/ultrasonic/range (Range) to /scan (LaserScan) for slam_toolbox."""

    def __init__(self) -> None:
        super().__init__("ultrasonic_to_scan_node")

        self._sub = self.create_subscription(
            Range, "/rover/ultrasonic/range", self._on_range, qos_profile_sensor_data
        )
        self._pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.get_logger().info("Ultrasonic → LaserScan bridge ready")

    def _on_range(self, msg: Range) -> None:
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = "base_link"

        # Single beam pointing straight ahead (0 rad).
        half_fov = msg.field_of_view / 2.0
        scan.angle_min = -half_fov
        scan.angle_max = half_fov
        scan.angle_increment = msg.field_of_view
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = msg.min_range
        scan.range_max = msg.max_range

        r = msg.range
        if math.isnan(r) or r < msg.min_range or r > msg.max_range:
            r = float("inf")
        scan.ranges = [r]

        self._pub.publish(scan)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = UltrasonicToScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
