import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class SlamPoseBridge(Node):
    """Republish ORB-SLAM3 PoseStamped as nav_msgs/Odometry and broadcast odom→base_link TF."""

    def __init__(self) -> None:
        super().__init__("slam_pose_bridge")

        self._sub = self.create_subscription(
            PoseStamped,
            "/orb_slam3/pose",
            self._on_pose,
            10,
        )
        self._pub = self.create_publisher(Odometry, "/rover/odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("SLAM pose bridge ready")

    def _on_pose(self, msg: PoseStamped) -> None:
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose = msg.pose
        self._pub.publish(odom)

        t = TransformStamped()
        t.header = msg.header
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self._tf_broadcaster.sendTransform(t)


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
