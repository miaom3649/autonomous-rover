import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# ORB-SLAM3 outputs poses in camera frame: z=forward, x=right, y=down.
# ROS robot frame (base_link):             x=forward, y=left,  z=up.
#
# Position mapping:
#   robot.x =  camera.z   (forward)
#   robot.y = -camera.x   (left = -right)
#   robot.z = -camera.y   (up   = -down)
#
# The equivalent fixed rotation matrix (camera → robot):
#   R = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
# As a unit quaternion (x, y, z, w):
_Q_FIX = (-0.5, 0.5, -0.5, 0.5)
_Q_FIX_INV = (0.5, -0.5, 0.5, 0.5)  # conjugate of _Q_FIX


def _qmul(q1: tuple, q2: tuple) -> tuple:
    """Hamilton product of two quaternions expressed as (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


class SlamPoseBridge(Node):
    """Republish ORB-SLAM3 PoseStamped as nav_msgs/Odometry and broadcast odom→base_link TF.

    Converts from ORB-SLAM3 camera frame (z=forward) to ROS robot frame (x=forward)
    so that Nav2 receives correctly oriented odometry.

    TF is published at a fixed rate (20 Hz) so Nav2 never loses the base_link frame.
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
        # Position: camera frame → robot frame
        cx = msg.pose.position.x
        cy = msg.pose.position.y
        cz = msg.pose.position.z
        rx = cz
        ry = -cx
        rz = -cy

        # Orientation: q_robot = q_fix * q_slam * q_fix_inv
        q_slam = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        qx, qy, qz, qw = _qmul(_Q_FIX, _qmul(q_slam, _Q_FIX_INV))

        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = rx
        odom.pose.pose.position.y = ry
        odom.pose.pose.position.z = rz
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self._pub.publish(odom)

        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = rx
        t.transform.translation.y = ry
        t.transform.translation.z = rz
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
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
        rclpy.shutdown()
