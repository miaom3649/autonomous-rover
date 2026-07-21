"""
Pure math for frame-to-frame visual odometry — no ROS dependencies, so this
is directly unit-testable (see tests/test_vo_pnp.py at the repo root).
Used by vo_node.py and slam_pose_bridge.py.
"""

import math
from typing import Optional

import cv2
import numpy as np

DEFAULT_MIN_INLIERS = 8
DEFAULT_MIN_DEPTH_SPREAD_M = 0.15
DEFAULT_MIN_PIXEL_FLOW_PX = 3.0
DEFAULT_CROSS_CHECK_TOLERANCE_M = 0.15
DEFAULT_CROSS_CHECK_TOLERANCE_RELATIVE = 0.4
DEFAULT_CROSS_CHECK_TOLERANCE_YAW_RAD = 0.26  # ~15 degrees


def unproject(
    u: float, v: float, depth: float, fx: float, fy: float, cx: float, cy: float
) -> np.ndarray:
    """Back-project a pixel + depth into a 3D point in the camera's optical
    frame (X-right, Y-down, Z-forward)."""
    return np.array([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], dtype=np.float64)


def is_well_conditioned(
    object_points: np.ndarray, min_depth_spread: float = DEFAULT_MIN_DEPTH_SPREAD_M
) -> bool:
    """PnP solves become numerically unstable when the 3D points are nearly
    coplanar — a well-known PnP degenerate configuration. A common case here
    is a flat wall filling most of the frame, where every matched point
    sits at nearly the same depth: tiny pixel-level match noise then gets
    amplified into huge or wildly wrong translation estimates. Reject such
    point sets up front rather than let PnP fit an unstable transform to
    them.
    """
    if len(object_points) < 4:
        return False
    depth_spread = float(object_points[:, 2].max() - object_points[:, 2].min())
    return depth_spread >= min_depth_spread


def median_pixel_flow(prev_points: np.ndarray, curr_points: np.ndarray) -> float:
    """Median matched-keypoint displacement (in pixels) between the two frames.

    PnP's translation estimate is only observable when there's real parallax
    between the two views — with near-zero true camera motion, the problem
    is close to singular and tiny match/pixel noise gets amplified into an
    arbitrarily large, arbitrarily-directed translation instead of a small
    one. Checking depth spread alone (is_well_conditioned) doesn't catch
    this: a scene can have plenty of depth variation and still be a
    degenerate PnP input if the camera itself barely moved between frames.
    A low median flow is direct, positive evidence the camera didn't move
    (rather than an absence-of-evidence fallback), so it's safe for the
    caller to report a confirmed zero-motion step instead of running PnP.
    """
    if len(prev_points) == 0:
        return 0.0
    return float(np.median(np.linalg.norm(curr_points - prev_points, axis=1)))


def solve_relative_pose(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    min_inliers: int = DEFAULT_MIN_INLIERS,
) -> Optional[tuple[np.ndarray, np.ndarray, int]]:
    """Solve PnP between 3D points from the previous frame and their observed
    2D positions in the current frame.

    Returns (R, t, num_inliers): the rigid transform mapping points from the
    previous camera's optical frame into the current camera's optical frame,
    or None if there are too few points or too few RANSAC inliers to trust
    the result.
    """
    if len(object_points) < 4:
        return None
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points.astype(np.float64),
        image_points.astype(np.float64),
        camera_matrix,
        dist_coeffs,
        reprojectionError=4.0,
        confidence=0.99,
        iterationsCount=200,
    )
    if not ok or inliers is None or len(inliers) < min_inliers:
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    return rotation, tvec.reshape(3), len(inliers)


def camera_delta_to_base_link_delta(
    rotation: np.ndarray, translation: np.ndarray
) -> tuple[float, float, float]:
    """Convert a previous-camera-frame -> current-camera-frame transform
    (as returned by solve_relative_pose) into a ground-plane (dx, dy, dtheta)
    delta in the robot's base_link convention (X-forward, Y-left, yaw about
    Z-up).

    The camera is mounted forward-facing and level, so the previous camera's
    optical frame (X-right, Y-down, Z-forward) is aligned with the robot's
    frame at the moment of the previous stop. The camera's own translation in
    that frame is C = -R^T t (the standard camera-center-in-object-frame
    formula). Yaw is read off the rotation's component about the camera's Y
    (down) axis, negated to match the robot's Z-up convention. Exact signs
    are verified empirically against real pushes, same as this project's
    established pattern for e.g. steering_sign in base_params.yaml.
    """
    center = -(rotation.T @ translation)
    dx = float(center[2])
    dy = float(-center[0])
    theta_cam = float(np.arctan2(rotation[0, 2], rotation[2, 2]))
    dtheta = -theta_cam
    return dx, dy, dtheta


def world_delta_to_local(
    prev_x: float, prev_y: float, prev_yaw: float, cur_x: float, cur_y: float, cur_yaw: float
) -> tuple[float, float, float]:
    """Convert two absolute (x, y, yaw) poses in the same world frame into a
    (dx, dy, dtheta) step expressed in the previous pose's local frame — the
    inverse of the compose-onto-a-running-pose step vo_node.py's
    _publish_delta does. Used by slam_pose_bridge.py to turn ORB-SLAM3's
    consecutive absolute poses into a step comparable against vo_node's own
    per-cycle (dx, dy, dtheta) measurement.
    """
    dx_world = cur_x - prev_x
    dy_world = cur_y - prev_y
    cos_p, sin_p = math.cos(prev_yaw), math.sin(prev_yaw)
    dx = dx_world * cos_p + dy_world * sin_p
    dy = -dx_world * sin_p + dy_world * cos_p
    dtheta = math.atan2(math.sin(cur_yaw - prev_yaw), math.cos(cur_yaw - prev_yaw))
    return dx, dy, dtheta


def deltas_agree(
    delta_a: tuple[float, float, float],
    delta_b: tuple[float, float, float],
    tolerance_m: float = DEFAULT_CROSS_CHECK_TOLERANCE_M,
    tolerance_relative: float = DEFAULT_CROSS_CHECK_TOLERANCE_RELATIVE,
    tolerance_yaw_rad: float = DEFAULT_CROSS_CHECK_TOLERANCE_YAW_RAD,
) -> bool:
    """Do two independently-measured (dx, dy, dtheta) steps for the same
    cycle roughly agree?

    Used to gate ORB-SLAM3 pose updates in slam_pose_bridge.py against
    vo_node.py's independent frame-to-frame PnP measurement: a genuine
    Bundle-Adjustment-artifact jump (SLAM3 retroactively/inconsistently
    rewriting its notion of the last step) won't match what incremental
    optical evidence actually measured, so it gets rejected instead of
    published. The tolerance is the larger of a flat floor (tolerance_m) and
    a fraction of the step size (tolerance_relative), since a fixed-size
    tolerance would be too loose for tiny steps and too tight for large ones.
    """
    dx_a, dy_a, dtheta_a = delta_a
    dx_b, dy_b, dtheta_b = delta_b
    translation_diff = math.hypot(dx_a - dx_b, dy_a - dy_b)
    reference = max(math.hypot(dx_a, dy_a), math.hypot(dx_b, dy_b))
    allowed_translation = max(tolerance_m, tolerance_relative * reference)
    yaw_diff = abs(math.atan2(math.sin(dtheta_a - dtheta_b), math.cos(dtheta_a - dtheta_b)))
    return translation_diff <= allowed_translation and yaw_diff <= tolerance_yaw_rad
