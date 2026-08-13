from rover_navigation.mapping_health import (
    HealthSample,
    HealthThresholds,
    angle_delta_deg,
    evaluate_sample,
)


def sample(**overrides):
    values = dict(stamp=1.0, x=0.0, y=0.0, yaw_deg=0.0, scan_age_s=0.1,
                  map_age_s=0.2, valid_scan_points=100, total_scan_points=200, tf_ok=True)
    values.update(overrides)
    return HealthSample(**values)


def test_angle_delta_wraps():
    assert angle_delta_deg(-179.0, 179.0) == 2.0


def test_healthy_sample_has_no_reasons():
    assert evaluate_sample(sample(), sample(x=-0.1), HealthThresholds()) == []


def test_reports_explicit_pose_and_sensor_failures():
    reasons = evaluate_sample(
        sample(x=2.0, scan_age_s=2.0, valid_scan_points=2, tf_ok=False),
        sample(), HealthThresholds(),
    )
    assert {reason["code"] for reason in reasons} == {
        "SCAN_TIMEOUT", "VALID_SCAN_TOO_LOW", "TF_TIMEOUT", "POSE_JUMP"
    }
