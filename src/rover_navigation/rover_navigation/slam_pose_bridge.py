import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class SlamPoseBridge(Node):
    """Republish ORB-SLAM3 PoseStamped as nav_msgs/Odometry and broadcast odom→base_link TF.

    TF is published at a fixed rate (20 Hz) so slam_toolbox never loses the base_link frame.
    Before SLAM initialises, broadcasts identity (rover at origin). After tracking is lost,
    holds the last known pose rather than dropping TF entirely.
    """

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

        self._last_t = self._make_identity()
        self.create_timer(0.05, self._publish_tf)  # 20 Hz keeps TF buffer fresh

        self.get_logger().info("SLAM pose bridge ready")

    def _make_identity(self) -> TransformStamped:
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.rotation.w = 1.0
        return t

    def _on_pose(self, msg: PoseStamped) -> None:
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose = msg.pose
        self._pub.publish(odom)

        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self._last_t = t

    def _publish_tf(self) -> None:
        self._last_t.header.stamp = self.get_clock().now().to_msg()
        self._tf_broadcaster.sendTransform(self._last_t)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SlamPoseBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
