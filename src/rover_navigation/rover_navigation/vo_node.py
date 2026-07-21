"""
Visual odometry node — runs on the Raspberry Pi.

Each time depth_bridge_node publishes a fresh, ultrasonic-scale-corrected
depth map (i.e. once per stop cycle — see depth_bridge_node.py), this node
matches ORB features between the current camera frame and the one cached
from the previous stop, looks up each matched point's real-world position
in the *previous* frame's depth map, and solves PnP: 3D points then vs. 2D
pixel positions now directly recovers the camera's metric relative motion
in one step (no separate essential-matrix-plus-scale-guess pass needed,
since every point already carries its own depth).

This node is *not* the rover's pose source — ORB-SLAM3 (orb_slam3_node) is,
via slam_pose_bridge.py. This node only measures each cycle's own
independent (dx, dy, dtheta) step and publishes it on /rover/vo_check.
slam_pose_bridge.py compares its own SLAM3-derived step for the same cycle
against this measurement before trusting it: ORB-SLAM3's Bundle Adjustment
can retroactively rewrite recent poses, producing a jump that this
node's honest, non-retroactive frame-to-frame measurement won't agree with
— see slam_pose_bridge.py's module docstring for the accept/reject logic.
This node itself never accumulates a running pose or owns any TF; it only
ever reports "this cycle's step, as far as I can tell."

A failed or degenerate step (too few features/matches, coplanar points,
PnP failure) publishes nothing to /rover/vo_check at all — a fabricated
"no motion" fallback would be indistinguishable from a real, confirmed
zero-flow measurement to slam_pose_bridge.py's cross-check, silently
disabling it exactly when there's no real evidence either way.
"""

from collections import deque
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rover_navigation.vo_math import (
    camera_delta_to_base_link_delta,
    is_well_conditioned,
    median_pixel_flow,
    solve_relative_pose,
    unproject,
)

DEFAULT_MIN_MATCHES = 15
DEFAULT_MIN_INLIERS = 8
DEFAULT_MAX_STEP_TRANSLATION_M = 1.0
DEFAULT_MAX_DEPTH_M = 6.0
DEFAULT_MATCH_RATIO = 0.75
DEFAULT_FRAME_MATCH_TOLERANCE_S = 0.15
DEFAULT_MIN_PIXEL_FLOW_PX = 3.0
RGB_BUFFER_SIZE = 20


class VoNode(Node):
    def __init__(self) -> None:
        super().__init__("vo_node")

        self.declare_parameter("fx", 321.03098008641035)
        self.declare_parameter("fy", 320.5022483982298)
        self.declare_parameter("cx", 156.38881537724336)
        self.declare_parameter("cy", 116.02029997042648)
        self.declare_parameter("k1", 0.34740589756056456)
        self.declare_parameter("k2", -2.328076843275508)
        self.declare_parameter("p1", -0.006620102436670903)
        self.declare_parameter("p2", -0.0031368694154414716)
        self.declare_parameter("min_matches", DEFAULT_MIN_MATCHES)
        self.declare_parameter("min_inliers", DEFAULT_MIN_INLIERS)
        self.declare_parameter("max_step_translation_m", DEFAULT_MAX_STEP_TRANSLATION_M)
        self.declare_parameter("max_depth_m", DEFAULT_MAX_DEPTH_M)
        self.declare_parameter("match_ratio", DEFAULT_MATCH_RATIO)
        self.declare_parameter("frame_match_tolerance_s", DEFAULT_FRAME_MATCH_TOLERANCE_S)
        self.declare_parameter("min_pixel_flow_px", DEFAULT_MIN_PIXEL_FLOW_PX)

        fx = float(self.get_parameter("fx").value)
        fy = float(self.get_parameter("fy").value)
        cx = float(self.get_parameter("cx").value)
        cy = float(self.get_parameter("cy").value)
        self._camera_matrix = np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64
        )
        self._dist_coeffs = np.array(
            [
                float(self.get_parameter("k1").value),
                float(self.get_parameter("k2").value),
                float(self.get_parameter("p1").value),
                float(self.get_parameter("p2").value),
            ],
            dtype=np.float64,
        )
        self._min_matches = int(self.get_parameter("min_matches").value)
        self._min_inliers = int(self.get_parameter("min_inliers").value)
        self._max_step = float(self.get_parameter("max_step_translation_m").value)
        self._max_depth = float(self.get_parameter("max_depth_m").value)
        self._match_ratio = float(self.get_parameter("match_ratio").value)
        self._frame_match_tolerance = float(self.get_parameter("frame_match_tolerance_s").value)
        self._min_pixel_flow_px = float(self.get_parameter("min_pixel_flow_px").value)

        self._bridge = CvBridge()
        self._orb = cv2.ORB_create(nfeatures=500)
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

        self._rgb_buffer: deque = deque(maxlen=RGB_BUFFER_SIZE)
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_depth: Optional[np.ndarray] = None
        self._prev_kp = None
        self._prev_des = None

        self._pub_vo_check = self.create_publisher(Odometry, "/rover/vo_check", 10)

        self.create_subscription(
            Image, "/rover/camera/image_raw", self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/rover/camera/depth", self._on_depth, qos_profile_sensor_data
        )

        self.get_logger().info("vo_node ready")

    def _on_image(self, msg: Image) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._rgb_buffer.append((stamp, self._bridge.imgmsg_to_cv2(msg, "rgb8")))

    def _find_matching_rgb(self, target_stamp: float) -> Optional[np.ndarray]:
        """depth_bridge_node's AI depth round-trip can take up to ~0.8s, during
        which newer camera frames may already have arrived — so "whatever RGB is
        cached right now" is not necessarily the same frame the depth map was
        computed from. Look up the buffered frame whose timestamp actually
        matches the depth message's (which carries the original frame's stamp,
        see depth_bridge_node.py), instead of silently pairing mismatched data.
        """
        if not self._rgb_buffer:
            return None
        best_stamp, best_rgb = min(self._rgb_buffer, key=lambda item: abs(item[0] - target_stamp))
        if abs(best_stamp - target_stamp) > self._frame_match_tolerance:
            return None
        return best_rgb

    def _on_depth(self, msg: Image) -> None:
        """Each new corrected depth map is a trigger: one VO step per stop cycle.

        Only publishes /rover/vo_check on a genuine successful measurement —
        see module docstring for why a failed step must publish nothing
        rather than a "no motion" fallback. The very first trigger has
        nothing to compare against yet, so it only caches and reports
        nothing either.
        """
        target_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rgb = self._find_matching_rgb(target_stamp)
        if rgb is None:
            self.get_logger().warn(
                "VO step skipped: no cached RGB frame matches this depth map's capture time"
            )
            return

        depth = self._bridge.imgmsg_to_cv2(msg, "32FC1")
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        kp, des = self._orb.detectAndCompute(gray, None)

        if self._prev_gray is None:
            self._prev_gray, self._prev_depth = gray, depth
            self._prev_kp, self._prev_des = kp, des
            return

        step = self._estimate_step(kp, des)

        self._prev_gray, self._prev_depth = gray, depth
        self._prev_kp, self._prev_des = kp, des

        if step is None:
            return
        dx, dy, dtheta = step
        self._publish_delta(msg.header.stamp, dx, dy, dtheta)

    def _estimate_step(self, kp, des) -> Optional[tuple[float, float, float]]:
        """Returns the (dx, dy, dtheta) step, or None on a degenerate/failed
        step. A None result means this cycle publishes nothing at all — see
        module docstring for why that matters."""
        if des is None or self._prev_des is None or len(des) < self._min_matches:
            self.get_logger().warn("VO step skipped: too few ORB features detected")
            return None

        matches = self._match_features(des)
        if len(matches) < self._min_matches:
            self.get_logger().warn("VO step skipped: too few ORB matches")
            return None

        prev_pts = np.array([self._prev_kp[m.queryIdx].pt for m in matches])
        curr_pts = np.array([kp[m.trainIdx].pt for m in matches])
        flow = median_pixel_flow(prev_pts, curr_pts)
        if flow < self._min_pixel_flow_px:
            # Real evidence the camera barely moved (not an absence-of-data
            # fallback) — see module docstring on median_pixel_flow in
            # vo_math.py. Reporting this confirms zero motion instead of
            # feeding a near-singular, near-zero-baseline problem to PnP,
            # which would otherwise blow up into an arbitrarily large,
            # arbitrarily-directed "displacement" from pure noise — and,
            # left unreported, would leave stop_and_go_filter_node waiting
            # forever for a fix that can never arrive once the rover is
            # actually stopped, since every future frame pair would be just
            # as motionless.
            self.get_logger().info(
                f"VO step: negligible pixel flow ({flow:.1f}px < {self._min_pixel_flow_px}px) "
                "— confirming no motion"
            )
            return 0.0, 0.0, 0.0

        fx, fy = self._camera_matrix[0, 0], self._camera_matrix[1, 1]
        cx, cy = self._camera_matrix[0, 2], self._camera_matrix[1, 2]
        object_points = []
        image_points = []
        for m in matches:
            u, v = self._prev_kp[m.queryIdx].pt
            d = self._sample_depth(self._prev_depth, u, v)
            if d is None:
                continue
            object_points.append(unproject(u, v, d, fx, fy, cx, cy))
            image_points.append(kp[m.trainIdx].pt)

        if len(object_points) < self._min_matches:
            self.get_logger().warn("VO step skipped: too few matches had valid depth")
            return None

        object_points_arr = np.array(object_points)
        if not is_well_conditioned(object_points_arr):
            self.get_logger().warn(
                "VO step skipped: matched points too close to coplanar "
                "(insufficient depth spread) for a stable PnP solve"
            )
            return None

        result = solve_relative_pose(
            object_points_arr,
            np.array(image_points),
            self._camera_matrix,
            self._dist_coeffs,
            self._min_inliers,
        )
        if result is None:
            self.get_logger().warn("VO step skipped: PnP failed / too few inliers")
            return None

        rotation, translation, _num_inliers = result
        dx, dy, dtheta = camera_delta_to_base_link_delta(rotation, translation)
        if np.hypot(dx, dy) > self._max_step:
            self.get_logger().warn(
                f"VO step skipped: implausible displacement "
                f"{np.hypot(dx, dy):.2f}m > {self._max_step}m"
            )
            return None
        return dx, dy, dtheta

    def _match_features(self, des) -> list:
        """Match against the previous frame's descriptors using Lowe's ratio
        test: a match is only kept if its best candidate is meaningfully
        closer than the second-best. Repetitive or low-texture-diversity
        scenes (tiled floors, patterned surfaces) otherwise produce
        confident-looking but wrong correspondences that a plain
        nearest-neighbor match (or crossCheck alone) won't catch, and PnP
        will happily fit an unstable, wrong transform to them.
        """
        knn_matches = self._matcher.knnMatch(self._prev_des, des, k=2)
        good_matches = []
        for pair in knn_matches:
            if len(pair) != 2:
                continue
            best, second = pair
            if best.distance < self._match_ratio * second.distance:
                good_matches.append(best)
        return good_matches

    def _sample_depth(self, depth: np.ndarray, u: float, v: float) -> Optional[float]:
        h, w = depth.shape
        iu, iv = int(round(u)), int(round(v))
        if not (0 <= iu < w and 0 <= iv < h):
            return None
        d = float(depth[iv, iu])
        if not np.isfinite(d) or d <= 0 or d > self._max_depth:
            return None
        return d

    def _publish_delta(self, stamp, dx: float, dy: float, dtheta: float) -> None:
        """Publish this cycle's independently-measured (dx, dy, dtheta) step
        on /rover/vo_check — a one-shot measurement, not an accumulated
        pose. slam_pose_bridge.py reads this to cross-check ORB-SLAM3's own
        step for the same cycle (matched by header.stamp, the original
        capture time) before trusting it."""
        qz, qw = np.sin(dtheta / 2.0), np.cos(dtheta / 2.0)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = dx
        msg.pose.pose.position.y = dy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self._pub_vo_check.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
