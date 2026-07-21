"""Unit tests for the pure PnP-based visual odometry math in vo_math.py.

No ROS2 install required — vo_math.py has no rclpy dependency (see its
module docstring), so it's imported directly by path here.
"""

import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), "..", "src", "rover_navigation", "rover_navigation"),
)

from vo_math import (  # noqa: E402
    camera_delta_to_base_link_delta,
    is_well_conditioned,
    solve_relative_pose,
    unproject,
)

CAMERA_MATRIX = np.array(
    [[321.03, 0.0, 156.39], [0.0, 320.50, 116.02], [0.0, 0.0, 1.0]], dtype=np.float64
)
ZERO_DIST = np.zeros(4, dtype=np.float64)


def _synthetic_points(n: int, rng: np.random.Generator) -> np.ndarray:
    """n random 3D points scattered in front of the camera (previous frame)."""
    x = rng.uniform(-1.0, 1.0, n)
    y = rng.uniform(-0.5, 0.5, n)
    z = rng.uniform(0.8, 3.0, n)
    return np.stack([x, y, z], axis=1)


def _project(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    rvec, _ = cv2.Rodrigues(rotation)
    image_points, _ = cv2.projectPoints(points, rvec, translation, CAMERA_MATRIX, ZERO_DIST)
    return image_points.reshape(-1, 2)


def _camera_motion_transform(
    center_delta: np.ndarray, yaw_cam: float
) -> tuple[np.ndarray, np.ndarray]:
    """Build the (R, t) previous->current transform for a camera that moved by
    `center_delta` (in the previous frame) and rotated by `yaw_cam` about its
    Y (down) axis — the inverse of what camera_delta_to_base_link_delta decodes."""
    rotation = np.array(
        [
            [np.cos(yaw_cam), 0.0, np.sin(yaw_cam)],
            [0.0, 1.0, 0.0],
            [-np.sin(yaw_cam), 0.0, np.cos(yaw_cam)],
        ]
    )
    translation = -rotation @ center_delta
    return rotation, translation


def test_solve_relative_pose_recovers_pure_forward_motion():
    rng = np.random.default_rng(0)
    points_prev = _synthetic_points(40, rng)
    rotation_true, translation_true = _camera_motion_transform(
        center_delta=np.array([0.0, 0.0, 0.2]), yaw_cam=0.0
    )
    points_curr = _project(points_prev, rotation_true, translation_true)

    result = solve_relative_pose(points_prev, points_curr, CAMERA_MATRIX, ZERO_DIST)
    assert result is not None
    rotation, translation, num_inliers = result
    assert num_inliers >= 30

    dx, dy, dtheta = camera_delta_to_base_link_delta(rotation, translation)
    assert dx == pytest.approx(0.2, abs=0.01)
    assert dy == pytest.approx(0.0, abs=0.01)
    assert dtheta == pytest.approx(0.0, abs=0.02)


def test_solve_relative_pose_recovers_turn_and_strafe():
    rng = np.random.default_rng(1)
    points_prev = _synthetic_points(60, rng)
    true_yaw = 0.15  # robot turns ~8.6 deg
    rotation_true, translation_true = _camera_motion_transform(
        center_delta=np.array([0.1, 0.0, 0.3]),
        yaw_cam=-true_yaw,
    )
    points_curr = _project(points_prev, rotation_true, translation_true)

    result = solve_relative_pose(points_prev, points_curr, CAMERA_MATRIX, ZERO_DIST)
    assert result is not None
    rotation, translation, _num_inliers = result

    dx, dy, dtheta = camera_delta_to_base_link_delta(rotation, translation)
    assert dx == pytest.approx(0.3, abs=0.01)
    assert dy == pytest.approx(-0.1, abs=0.01)
    assert dtheta == pytest.approx(true_yaw, abs=0.02)


def test_unproject_center_pixel_is_pure_depth():
    fx, fy, cx, cy = 300.0, 300.0, 160.0, 120.0
    point = unproject(cx, cy, 2.5, fx, fy, cx, cy)
    assert point[0] == pytest.approx(0.0)
    assert point[1] == pytest.approx(0.0)
    assert point[2] == pytest.approx(2.5)


def test_solve_relative_pose_returns_none_with_too_few_points():
    points_prev = np.zeros((3, 3))
    points_curr = np.zeros((3, 2))
    assert solve_relative_pose(points_prev, points_curr, CAMERA_MATRIX, ZERO_DIST) is None


def test_is_well_conditioned_rejects_near_planar_points():
    rng = np.random.default_rng(2)
    x = rng.uniform(-1.0, 1.0, 40)
    y = rng.uniform(-0.5, 0.5, 40)
    z = np.full(40, 2.0) + rng.uniform(-0.01, 0.01, 40)  # a flat wall, ~2m out
    flat_points = np.stack([x, y, z], axis=1)
    assert is_well_conditioned(flat_points) is False


def test_is_well_conditioned_accepts_points_with_real_depth_spread():
    points_prev = _synthetic_points(40, np.random.default_rng(3))
    assert is_well_conditioned(points_prev) is True
