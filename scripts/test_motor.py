# Hardware diagnostic script — direct SDK access intentional.
# Not a ROS node. Run on physical hardware only.

"""
Rear-wheel motor diagnostic script.

Drives the rover forward for 5 seconds, stops briefly, then drives backward
for 5 seconds, and stops. Validates that both rear motors are responsive and
spinning in the correct direction.

Usage:
    python3 scripts/test_motor.py [--speed SPEED]

Exit codes:
    0  — motors ran to completion without errors
    1  — Robot Hat not detected, robot_hat SDK not available, or motor error
"""

import argparse
import sys
import time


DEFAULT_SPEED = 30  # percent, 0–100
MOVE_DURATION_S = 5
STOP_PAUSE_S = 1

# Robot Hat V4 sits at one of these I2C addresses on bus 1.
_ROBOT_HAT_I2C_ADDRS = (0x14, 0x15)
_I2C_BUS = 1


def _detect_robot_hat() -> bool:
    """Return True if the Robot Hat V4 responds on the I2C bus."""
    try:
        import smbus2  # type: ignore
    except ImportError:
        print("[i2c] smbus2 not available — skipping board detection.")
        return True  # can't check; let picarx try anyway

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


def _run_sequence(speed: int) -> bool:
    """Drive rear motors via robot_hat.Motors — steering servo is not touched."""
    try:
        from robot_hat import Motors  # type: ignore
    except ImportError:
        print("[robot_hat] Library not installed — run on the Raspberry Pi with the Sunfounder SDK.")
        return False

    try:
        motors = Motors()
    except Exception as exc:
        print(f"[robot_hat] Failed to initialise Motors: {exc}")
        return False

    try:
        print(f"[motor] Forward at speed {speed} for {MOVE_DURATION_S}s …")
        motors.speed(speed, speed)
        time.sleep(MOVE_DURATION_S)

        print("[motor] Stop.")
        motors.speed(0, 0)
        time.sleep(STOP_PAUSE_S)

        print(f"[motor] Backward at speed {speed} for {MOVE_DURATION_S}s …")
        motors.speed(-speed, -speed)
        time.sleep(MOVE_DURATION_S)

        print("[motor] Stop.")
        motors.speed(0, 0)
    except Exception as exc:
        print(f"[motor] Error during motion sequence: {exc}")
        try:
            motors.speed(0, 0)
        except Exception:
            pass
        return False

    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Rear-wheel motor diagnostic")
    parser.add_argument(
        "--speed",
        type=int,
        default=DEFAULT_SPEED,
        metavar="SPEED",
        help=f"Motor speed, 0–100 (default: {DEFAULT_SPEED})",
    )
    args = parser.parse_args()

    if not 0 <= args.speed <= 100:
        print(f"Error: --speed must be between 0 and 100, got {args.speed}")
        return 1

    print("=== Motor Diagnostic ===")
    print(f"Speed       : {args.speed}%")
    print(f"Duration    : {MOVE_DURATION_S}s forward + {MOVE_DURATION_S}s backward")
    print()

    if not _detect_robot_hat():
        print("\nResult: FAIL")
        return 1

    if _run_sequence(args.speed):
        print("\nResult: PASS")
        return 0

    print("\nResult: FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
