# Hardware diagnostic script — direct SDK access intentional.
# Not a ROS node. Run on physical hardware only.

"""
Ultrasonic distance sensor diagnostic script.

Continuously prints distance readings from the HC-SR04-compatible sensor on
the Robot Hat V4. Press Ctrl-C to stop. Prints a summary (min/max/avg) on exit.

Usage:
    python3 scripts/test_ultrasonic.py [--interval SECONDS] [--count N]

Exit codes:
    0  — ran to completion (or stopped by Ctrl-C) without errors
    1  — Robot Hat not detected, robot_hat SDK not available, or sensor error
"""

import argparse
import sys
import time


DEFAULT_INTERVAL_S = 0.1   # 10 Hz
DEFAULT_COUNT = 0           # 0 = run until Ctrl-C

# Robot Hat V4 sits at one of these I2C addresses on bus 1.
_ROBOT_HAT_I2C_ADDRS = (0x14, 0x15)
_I2C_BUS = 1

# Ultrasonic trigger/echo pins on Robot Hat V4.
_TRIG_PIN = "D2"
_ECHO_PIN = "D3"

# Readings outside this range are likely sensor errors.
_MIN_VALID_CM = 2.0
_MAX_VALID_CM = 400.0


def _detect_robot_hat() -> bool:
    """Return True if the Robot Hat V4 responds on the I2C bus."""
    try:
        import smbus2  # type: ignore
    except ImportError:
        print("[i2c] smbus2 not available — skipping board detection.")
        return True

    bus = smbus2.SMBus(_I2C_BUS)
    try:
        for addr in _ROBOT_HAT_I2C_ADDRS:
            try:
                bus.read_byte(addr)
                print(f"[i2c] Robot Hat detected at address 0x{addr:02X}.")
                return True
            except OSError:
                continue
        print(
            f"[i2c] No response at addresses "
            f"{', '.join(f'0x{a:02X}' for a in _ROBOT_HAT_I2C_ADDRS)} — "
            "is the driver board powered and connected?"
        )
        return False
    finally:
        bus.close()


def _init_sensor():
    """Return an Ultrasonic sensor instance or None on failure."""
    try:
        from robot_hat import Ultrasonic, Pin  # type: ignore
    except ImportError:
        print("[robot_hat] Library not installed — run on the Raspberry Pi with the Sunfounder SDK.")
        return None
    try:
        sensor = Ultrasonic(Pin(_TRIG_PIN), Pin(_ECHO_PIN))
        return sensor
    except Exception as exc:
        print(f"[robot_hat] Failed to initialise ultrasonic sensor: {exc}")
        return None


def _run_readings(sensor, interval: float, count: int) -> bool:
    """Print distance readings until count is reached or Ctrl-C is pressed."""
    readings: list[float] = []
    n = 0

    print(f"{'#':>6}  {'Distance (cm)':>14}  {'Status'}")
    print("-" * 35)

    try:
        while count == 0 or n < count:
            raw = sensor.read()
            n += 1

            if raw is None or raw < _MIN_VALID_CM or raw > _MAX_VALID_CM:
                status = "OUT OF RANGE" if raw is not None else "NO ECHO"
                print(f"{n:>6}  {'---':>14}  {status}")
            else:
                readings.append(raw)
                print(f"{n:>6}  {raw:>13.1f}  OK")

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n[stopped by Ctrl-C]")

    _print_summary(readings, n)
    return True


def _print_summary(readings: list[float], total: int) -> None:
    valid = len(readings)
    print()
    print("=== Summary ===")
    print(f"Total readings : {total}")
    print(f"Valid readings : {valid}")
    if valid == 0:
        print("No valid readings — check sensor wiring.")
        return
    print(f"Min            : {min(readings):.1f} cm")
    print(f"Max            : {max(readings):.1f} cm")
    print(f"Avg            : {sum(readings) / valid:.1f} cm")


def main() -> int:
    parser = argparse.ArgumentParser(description="Ultrasonic distance sensor diagnostic")
    parser.add_argument(
        "--interval",
        type=float,
        default=DEFAULT_INTERVAL_S,
        metavar="SECONDS",
        help=f"Sampling interval in seconds (default: {DEFAULT_INTERVAL_S})",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=DEFAULT_COUNT,
        metavar="N",
        help="Number of readings to take, 0 = run until Ctrl-C (default: 0)",
    )
    args = parser.parse_args()

    if args.interval <= 0:
        print(f"Error: --interval must be positive, got {args.interval}")
        return 1
    if args.count < 0:
        print(f"Error: --count must be >= 0, got {args.count}")
        return 1

    print("=== Ultrasonic Sensor Diagnostic ===")
    print(f"Pins           : TRIG={_TRIG_PIN}  ECHO={_ECHO_PIN}")
    print(f"Sample interval: {args.interval}s  ({1 / args.interval:.0f} Hz)")
    print(f"Count          : {'unlimited' if args.count == 0 else args.count}")
    print()

    if not _detect_robot_hat():
        print("\nResult: FAIL")
        return 1

    sensor = _init_sensor()
    if sensor is None:
        print("\nResult: FAIL")
        return 1

    if _run_readings(sensor, args.interval, args.count):
        print("\nResult: PASS")
        return 0

    print("\nResult: FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
