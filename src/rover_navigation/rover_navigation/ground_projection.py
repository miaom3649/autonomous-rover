"""Project image pixels onto a flat ground plane.

Camera convention: OpenCV optical frame (x right, y down, z forward).
Rover convention: ROS base_link (x forward, y left, z up).
"""

import math


def pixel_to_ground(
    u: float,
    v: float,
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    camera_height_m: float,
    camera_x_m: float = 0.0,
    camera_y_m: float = 0.0,
    camera_pitch_down_deg: float = 0.0,
    camera_yaw_left_deg: float = 0.0,
) -> tuple[float, float, float] | None:
    """Return the ground intersection in ``base_link`` as ``(x, y, 0)``.

    ``camera_pitch_down_deg`` is positive when the optical axis points below
    the horizon. ``camera_yaw_left_deg`` is positive when it points left.
    Returns ``None`` when the pixel ray does not hit the ground in front of
    the camera.
    """
    if fx <= 0.0 or fy <= 0.0 or camera_height_m <= 0.0:
        raise ValueError("fx, fy, and camera_height_m must be positive")

    x_optical = (u - cx) / fx
    y_optical = (v - cy) / fy
    pitch = math.radians(camera_pitch_down_deg)

    # Optical ray [right, down, forward] -> level base_link, with pitch.
    ray_x = math.cos(pitch) + y_optical * math.sin(pitch)
    ray_y = -x_optical
    ray_z = -math.sin(pitch) - y_optical * math.cos(pitch)
    if ray_z >= -1e-9:
        return None

    distance_scale = camera_height_m / -ray_z
    forward = distance_scale * ray_x
    left = distance_scale * ray_y
    if forward <= 0.0:
        return None

    # Apply camera pan/yaw after the ground intersection is found.
    yaw = math.radians(camera_yaw_left_deg)
    base_x = camera_x_m + math.cos(yaw) * forward - math.sin(yaw) * left
    base_y = camera_y_m + math.sin(yaw) * forward + math.cos(yaw) * left
    return (base_x, base_y, 0.0)
