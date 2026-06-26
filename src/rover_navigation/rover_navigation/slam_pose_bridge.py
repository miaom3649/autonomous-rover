import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def _qmul(q1: tuple, q2: tuple) -> tuple:
    """Hamilton product of two quaternions (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _qinv(q: tuple) -> tuple:
    """Conjugate (= inverse for unit quaternions)."""
    return (-q[0], -q[1], -q[2], q[3])


def _qrot(q: tuple, v: tuple) -> tuple:
    """Rotate vector v = (x, y, z) by unit quaternion q. Returns (x', y', z')."""
    qv = (v[0], v[1], v[2], 0.0)
    r = _qmul(_qmul(q, qv), _qinv(q))
    return r[0], r[1], r[2]


# ──────────────────────────────────────────────────────────────────────────────
# Coordinate-frame convention
# ──────────────────────────────────────────────────────────────────────────────
# ORB-SLAM3 outputs poses in the camera's own frame at initialisation time:
#   camera.z = optical axis (forward)
#   camera.x = right
#   camera.y = down
#
# PiCar-X camera servo convention (matches calibrate_camera_tilt.py):
#   tilt > 0  → camera points UP   (default was +7°)
#   tilt < 0  → camera points DOWN (set to -25° for floor-facing SLAM)
#   pan  > 0  → camera turns RIGHT (set to +13°, treated as yaw offset)
#
# This node builds a combined rotation quaternion q_fix that transforms
# from the SLAM world frame into the ROS base_link frame (x=fwd, y=left, z=up).
# q_fix = q_base ∘ q_tilt, where:
#   q_base  encodes the base camera→robot reorientation (z→x, -x→y, -y→z)
#   q_tilt  encodes the servo tilt around the camera x-axis
# ──────────────────────────────────────────────────────────────────────────────

# Base rotation: camera(z=fwd,x=right,y=down) → robot(x=fwd,y=left,z=up)
# Rotation matrix R = [[0,0,1],[-1,0,0],[0,-1,0]] → quaternion (x,y,z,w)
_Q_BASE = (-0.5, 0.5, -0.5, 0.5)


def _build_fix(tilt_deg: float) -> tuple:
    """Return the camera-to-robot quaternion for a given servo tilt angle.

    tilt_deg: PiCar-X tilt in degrees (positive = up, negative = down).
    """
    half = math.radians(tilt_deg) / 2.0
    # Rotation around camera x-axis by tilt_deg.
    q_tilt = (math.sin(half), 0.0, 0.0, math.cos(half))
    return _qmul(_Q_BASE, q_tilt)


class SlamPoseBridge(Node):
    """Convert ORB-SLAM3 PoseStamped → Odometry + odom→base_link TF.

    Accounts for:
    - ORB-SLAM3 camera frame convention (z=forward, x=right, y=down)
    - Physical camera servo tilt angle (camera_tilt_deg parameter)

    TF is broadcast at 20 Hz so Nav2 never loses base_link. Identity is held
    until SLAM initialises; last known pose is held after tracking loss.
    """

    def __init__(self) -> None:
        super().__init__("slam_pose_bridge")

        self.declare_parameter("camera_tilt_deg", -13.0)
        tilt = float(self.get_parameter("camera_tilt_deg").value)

        self._q_fix = _build_fix(tilt)
        self._q_fix_inv = _qinv(self._q_fix)
        self.get_logger().info(f"Camera frame correction: tilt={tilt:+.1f}°")

        self._sub = self.create_subscription(
            PoseStamped, "/orb_slam3/pose", self._on_pose, 10)
        self._pub = self.create_publisher(Odometry, "/rover/odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._last_t = self._make_identity()
        self.create_timer(0.05, self._publish_tf)

        self.get_logger().info("SLAM pose bridge ready")

    def _make_identity(self) -> TransformStamped:
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.rotation.w = 1.0
        return t

    def _on_pose(self, msg: PoseStamped) -> None:
        # ORB-SLAM3 returns Tcw (world→camera). Invert to get Twc (camera pose in world).
        q_cw = (msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w)
        q_wc = _qinv(q_cw)

        # Camera world position: P_world = R_wc * (−t_cw)
        cx = msg.pose.position.x
        cy = msg.pose.position.y
        cz = msg.pose.position.z
        pwx, pwy, pwz = _qrot(q_wc, (-cx, -cy, -cz))

        # Transform from SLAM world frame into ROS robot frame.
        rx, ry, rz = _qrot(self._q_fix, (pwx, pwy, pwz))
        qx, qy, qz, qw = _qmul(self._q_fix, _qmul(q_wc, self._q_fix_inv))

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
