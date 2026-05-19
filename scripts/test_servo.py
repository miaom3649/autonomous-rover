# Hardware diagnostic script — direct SDK access intentional.
# Not a ROS node. Run on physical hardware only.

"""
Steering servo diagnostic script.

Sweeps the front-wheel steering servo: center → left → center → right → center.
Validates that the servo responds and turns smoothly across its range.

Usage:
    python3 scripts/test_servo.py [--angle ANGLE]

Exit codes:
    0  — sweep completed without errors
    1  — Robot Hat not detected, picarx SDK not available, or servo error
"""

import argparse
import sys
import time


DEFAULT_ANGLE = 30  # degrees, max deflection from center
HOLD_S = 2          # seconds to hold each position

# Robot Hat V4 sits at one of these I2C addresses on bus 1.
_ROBOT_HAT_I2C_ADDRS = (0x14, 0x15)
_I2C_BUS = 1


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


def _run_sweep(angle: int) -> bool:
    """Initialise picarx and sweep steering servo through center/left/right."""
    try:
        from picarx import Picarx  # type: ignore
    except ImportError:
        print("[picarx] Library not installed — run on the Raspberry Pi with the Sunfounder SDK.")
        return False

    try:
        px = Picarx()
    except Exception as exc:
        print(f"[picarx] Failed to initialise: {exc}")
        return False

    steps = [
        (0,      "Center"),
        (-angle, "Left"),
        (0,      "Center"),
        (angle,  "Right"),
        (0,      "Center"),
    ]

    try:
        for deg, label in steps:
            print(f"[servo] {label} ({deg:+d}°) — holding {HOLD_S}s …")
            px.set_dir_servo_angle(deg)
            time.sleep(HOLD_S)
    except Exception as exc:
        print(f"[servo] Error during sweep: {exc}")
        try:
            px.set_dir_servo_angle(0)
        except Exception:
            pass
        return False
    finally:
        px.set_dir_servo_angle(0)

    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Steering servo diagnostic")
    parser.add_argument(
        "--angle",
        type=int,
        default=DEFAULT_ANGLE,
        metavar="ANGLE",
        help=f"Max deflection in degrees (default: {DEFAULT_ANGLE})",
    )
    args = parser.parse_args()

    if not 0 < args.angle <= 45:
        print(f"Error: --angle must be between 1 and 45, got {args.angle}")
        return 1

    print("=== Servo Diagnostic ===")
    print(f"Max angle   : ±{args.angle}°")
    print(f"Hold time   : {HOLD_S}s per position")
    print()

    if not _detect_robot_hat():
        print("\nResult: FAIL")
        return 1

    if _run_sweep(args.angle):
        print("\nResult: PASS")
        return 0

    print("\nResult: FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
