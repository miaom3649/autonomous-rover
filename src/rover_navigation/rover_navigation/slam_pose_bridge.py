import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class SlamPoseBridge(Node):
    """Republish ORB-SLAM3 PoseStamped as nav_msgs/Odometry on /rover/odom."""

    def __init__(self) -> None:
        super().__init__("slam_pose_bridge")

        self._sub = self.create_subscription(
            PoseStamped,
            "/orb_slam3/pose",
            self._on_pose,
            10,
        )
        self._pub = self.create_publisher(Odometry, "/rover/odom", 10)

        self.get_logger().info("SLAM pose bridge ready")

    def _on_pose(self, msg: PoseStamped) -> None:
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = "base_link"
        odom.pose.pose = msg.pose
        self._pub.publish(odom)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SlamPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
