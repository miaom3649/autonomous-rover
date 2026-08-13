"""ROS-independent mapping-health evaluation helpers."""

from dataclasses import dataclass, field
import math


def angle_delta_deg(current: float, previous: float) -> float:
    return abs((current - previous + 180.0) % 360.0 - 180.0)


@dataclass
class HealthSample:
    stamp: float
    x: float | None
    y: float | None
    yaw_deg: float | None
    scan_age_s: float
    map_age_s: float
    valid_scan_points: int
    total_scan_points: int
    tf_ok: bool
    reasons: list[dict] = field(default_factory=list)


@dataclass(frozen=True)
class HealthThresholds:
    pose_jump_m: float = 0.5
    yaw_jump_deg: float = 35.0
    scan_timeout_s: float = 1.0
    map_timeout_s: float = 3.0
    min_valid_scan_points: int = 30


def evaluate_sample(
    sample: HealthSample,
    previous: HealthSample | None,
    thresholds: HealthThresholds,
) -> list[dict]:
    """Return explicit reason records for an unhealthy sample."""
    reasons: list[dict] = []
    if sample.scan_age_s > thresholds.scan_timeout_s:
        reasons.append({"code": "SCAN_TIMEOUT", "value": sample.scan_age_s,
                        "threshold": thresholds.scan_timeout_s, "unit": "s"})
    if sample.map_age_s > thresholds.map_timeout_s:
        reasons.append({"code": "MAP_TIMEOUT", "value": sample.map_age_s,
                        "threshold": thresholds.map_timeout_s, "unit": "s"})
    if sample.valid_scan_points < thresholds.min_valid_scan_points:
        reasons.append({"code": "VALID_SCAN_TOO_LOW", "value": sample.valid_scan_points,
                        "threshold": thresholds.min_valid_scan_points, "unit": "points"})
    if not sample.tf_ok:
        reasons.append({"code": "TF_TIMEOUT", "value": 0, "threshold": 1, "unit": "available"})
    if (
        previous is not None
        and sample.x is not None and sample.y is not None
        and previous.x is not None and previous.y is not None
    ):
        jump = math.hypot(sample.x - previous.x, sample.y - previous.y)
        if jump > thresholds.pose_jump_m:
            reasons.append({"code": "POSE_JUMP", "value": jump,
                            "threshold": thresholds.pose_jump_m, "unit": "m"})
        if sample.yaw_deg is not None and previous.yaw_deg is not None:
            yaw_jump = angle_delta_deg(sample.yaw_deg, previous.yaw_deg)
            if yaw_jump > thresholds.yaw_jump_deg:
                reasons.append({"code": "YAW_JUMP", "value": yaw_jump,
                                "threshold": thresholds.yaw_jump_deg, "unit": "deg"})
    return reasons
