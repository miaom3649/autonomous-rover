"""
Pure math for frame-to-frame visual odometry — no ROS dependencies, so this
is directly unit-testable (see tests/test_vo_pnp.py at the repo root).
Used by vo_node.py.
"""

from typing import Optional

import cv2
import numpy as np

DEFAULT_MIN_INLIERS = 8


def unproject(
    u: float, v: float, depth: float, fx: float, fy: float, cx: float, cy: float
) -> np.ndarray:
    """Back-project a pixel + depth into a 3D point in the camera's optical
    frame (X-right, Y-down, Z-forward)."""
    return np.array([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], dtype=np.float64)


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
