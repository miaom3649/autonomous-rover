"""Unit tests for the pure PnP-based visual odometry math in vo_math.py.

No ROS2 install required — vo_math.py has no rclpy dependency (see its
module docstring), so it's imported directly by path here.
"""

import math
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
    deltas_agree,
    is_well_conditioned,
    median_pixel_flow,
    solve_relative_pose,
    unproject,
    world_delta_to_local,
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


def test_median_pixel_flow_zero_for_identical_points():
    pts = np.array([[10.0, 20.0], [30.0, 40.0], [50.0, 60.0]])
    assert median_pixel_flow(pts, pts) == pytest.approx(0.0)


def test_median_pixel_flow_matches_known_displacement():
    prev_pts = np.array([[0.0, 0.0], [10.0, 0.0], [0.0, 10.0], [10.0, 10.0]])
    curr_pts = prev_pts + np.array([3.0, 4.0])  # every point shifts by 5px
    assert median_pixel_flow(prev_pts, curr_pts) == pytest.approx(5.0)


def test_world_delta_to_local_pure_forward_motion():
    # Facing +45 degrees, moving 1m along that heading in world coordinates
    # should read back as pure forward (dx=1, dy=0) in the local frame.
    yaw = math.pi / 4
    prev = (0.0, 0.0, yaw)
    cur = (math.cos(yaw), math.sin(yaw), yaw)
    dx, dy, dtheta = world_delta_to_local(*prev, *cur)
    assert dx == pytest.approx(1.0)
    assert dy == pytest.approx(0.0, abs=1e-9)
    assert dtheta == pytest.approx(0.0)


def test_world_delta_to_local_pure_rotation():
    dx, dy, dtheta = world_delta_to_local(1.0, 2.0, 0.1, 1.0, 2.0, 0.1 + 0.3)
    assert dx == pytest.approx(0.0)
    assert dy == pytest.approx(0.0)
    assert dtheta == pytest.approx(0.3)


def test_deltas_agree_accepts_close_matches():
    assert deltas_agree((0.20, 0.02, 0.01), (0.22, 0.00, 0.02)) is True


def test_deltas_agree_rejects_ba_style_jump():
    # A genuine ~0.2m step vs. a multi-meter reported jump — the exact
    # failure mode this check exists to catch.
    assert deltas_agree((0.20, 0.0, 0.0), (4.50, 0.0, 0.0)) is False


def test_deltas_agree_rejects_large_yaw_disagreement():
    assert deltas_agree((0.20, 0.0, 0.0), (0.20, 0.0, 1.2)) is False
