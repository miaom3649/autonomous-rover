# Hardware diagnostic script — direct SDK access intentional.
# Not a ROS node. Run on physical hardware only.

"""
Kalman filter position estimator demo.

Reads distance samples from the ultrasonic sensor, runs them through the 1D
Kalman filter (position-only state), and saves a plot comparing raw readings
against the filtered output.

The car should be stationary and facing a wall during this test.

Usage:
    python3 scripts/demo_kalman_filter.py [--samples N] [--rate HZ]

Exit codes:
    0  — plot saved successfully
    1  — sensor error or missing dependency
"""

import argparse
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


DEFAULT_SAMPLES = 100
DEFAULT_RATE_HZ = 10
DEFAULT_OUTPUT = Path("/tmp/kalman_demo.png")
DEV_LOG_DIR = "~/dev/autonomous-rover/log"

# Kalman filter tuning — same defaults as position_estimator_node.
Q_POS = 0.01   # process noise variance (m^2)
R_POS = 0.05   # measurement noise variance (m^2)

_TRIG_PIN = "D2"
_ECHO_PIN = "D3"
_MIN_RANGE_M = 0.02
_MAX_RANGE_M = 4.00


# ---------------------------------------------------------------------------
# Kalman filter (standalone copy — no ROS dependency)
# ---------------------------------------------------------------------------

class KalmanFilter1D:
    """1D Kalman filter: single state (position), velocity as control input."""

    def __init__(self, q_pos: float, r_pos: float) -> None:
        self._x: float = 0.0
        self._P: float = 10.0
        self._Q: float = q_pos
        self._R: float = r_pos
        self._initialized = False

    def initialize(self, position: float) -> None:
        self._x = position
        self._initialized = True

    def predict(self, velocity_cmd: float, dt: float) -> None:
        self._x = self._x + velocity_cmd * dt
        self._P = self._P + self._Q

    def update(self, position_measurement: float) -> None:
        y = position_measurement - self._x
        S = self._P + self._R
        K = self._P / S
        self._x = self._x + K * y
        self._P = (1.0 - K) * self._P

    @property
    def position(self) -> float:
        return self._x


# ---------------------------------------------------------------------------
# Sensor reading
# ---------------------------------------------------------------------------

def _init_sensor():
    try:
        from robot_hat import Pin, Ultrasonic  # type: ignore
    except ImportError:
        print("[robot_hat] Library not installed — run on the Raspberry Pi.")
        return None
    try:
        return Ultrasonic(Pin(_TRIG_PIN), Pin(_ECHO_PIN))
    except Exception as exc:
        print(f"[robot_hat] Failed to initialise sensor: {exc}")
        return None


def _read_distance(sensor) -> float | None:
    try:
        raw_cm = sensor.read()
    except Exception as exc:
        print(f"[sensor] Read error: {exc}")
        return None
    if raw_cm is None:
        return None
    distance_m = raw_cm / 100.0
    if not (_MIN_RANGE_M <= distance_m <= _MAX_RANGE_M):
        return None
    return distance_m


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def _save_plot(timestamps, raw_readings, filtered_readings, output: Path) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")   # headless — no display needed on Pi
        import matplotlib.pyplot as plt
    except ImportError:
        print("[matplotlib] Not installed. Run: pip install matplotlib")
        return False

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(timestamps, raw_readings, color="tab:blue", alpha=0.5,
            linewidth=1, label="Raw ultrasonic (m)")
    ax.plot(timestamps, filtered_readings, color="tab:orange",
            linewidth=2, label="Kalman filter output (m)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance to wall (m)")
    ax.set_title("1D Kalman Filter — Position Estimation")
    ax.legend()
    ax.grid(True, linestyle="--", alpha=0.5)
    fig.tight_layout()

    try:
        fig.savefig(str(output), dpi=150)
        plt.close(fig)
        print(f"[plot] Saved to: {output.resolve()}")
        return True
    except Exception as exc:
        print(f"[plot] Failed to save: {exc}")
        return False


# ---------------------------------------------------------------------------
# SCP back to dev machine
# ---------------------------------------------------------------------------

def _scp_to_dev(local_path: Path) -> None:
    ssh_client_env = os.environ.get("SSH_CLIENT", "").split()
    if not ssh_client_env:
        print(f"Plot saved locally: {local_path.resolve()}")
        return

    dev_ip = ssh_client_env[0]
    dev_user = os.environ.get("ROVER_DEV_USER", "konkon")
    remote_dir = f"{dev_user}@{dev_ip}:{DEV_LOG_DIR}/"

    result = subprocess.run(
        ["scp", str(local_path), remote_dir],
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        print(f"[scp] Plot copied to dev machine: {DEV_LOG_DIR}/{local_path.name}")
    else:
        print(f"[scp] Failed: {result.stderr.strip()}")
        print(f"      Plot saved locally on Pi: {local_path.resolve()}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description="Kalman filter position estimator demo")
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES,
                        help=f"Number of samples to collect (default: {DEFAULT_SAMPLES})")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ,
                        help=f"Sampling rate in Hz (default: {DEFAULT_RATE_HZ})")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT,
                        help=f"Output PNG path (default: {DEFAULT_OUTPUT})")
    args = parser.parse_args()

    period_s = 1.0 / args.rate
    duration_s = args.samples / args.rate

    print("=== Kalman Filter Demo ===")
    print(f"Samples     : {args.samples}")
    print(f"Rate        : {args.rate} Hz")
    print(f"Duration    : {duration_s:.1f} s")
    print(f"q_pos       : {Q_POS}  (process noise)")
    print(f"r_pos       : {R_POS}  (measurement noise)")
    print()

    sensor = _init_sensor()
    if sensor is None:
        return 1

    kf = KalmanFilter1D(q_pos=Q_POS, r_pos=R_POS)

    timestamps: list[float] = []
    raw_readings: list[float] = []
    filtered_readings: list[float] = []

    print(f"Collecting {args.samples} samples — keep the car still, facing the wall.")
    print()

    start = time.monotonic()
    skipped = 0

    for i in range(args.samples):
        t_target = start + i * period_s
        now = time.monotonic()
        if t_target > now:
            time.sleep(t_target - now)

        distance_m = _read_distance(sensor)
        if distance_m is None:
            skipped += 1
            continue

        t = time.monotonic() - start

        if not kf._initialized:
            kf.initialize(distance_m)
        else:
            kf.predict(velocity_cmd=0.0, dt=period_s)
            kf.update(distance_m)

        timestamps.append(t)
        raw_readings.append(distance_m)
        filtered_readings.append(kf.position)

        print(f"  [{t:6.2f}s] raw={distance_m:.3f} m  filtered={kf.position:.3f} m")

    print()
    if skipped:
        print(f"Skipped {skipped} invalid readings.")

    if len(raw_readings) < 2:
        print("Not enough valid readings to plot.")
        return 1

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output.with_stem(f"kalman_demo_{timestamp_str}")

    if not _save_plot(timestamps, raw_readings, filtered_readings, output):
        return 1

    _scp_to_dev(output)
    print("\nResult: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
