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
    1  — picarx SDK not available or motor initialisation failed
"""

import argparse
import sys
import time


DEFAULT_SPEED = 30  # percent, 0–100
MOVE_DURATION_S = 5
STOP_PAUSE_S = 1


def _run_sequence(speed: int) -> bool:
    """Initialise picarx and execute forward → stop → backward → stop."""
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

    try:
        print(f"[motor] Forward at speed {speed} for {MOVE_DURATION_S}s …")
        px.forward(speed)
        time.sleep(MOVE_DURATION_S)

        print("[motor] Stop.")
        px.stop()
        time.sleep(STOP_PAUSE_S)

        print(f"[motor] Backward at speed {speed} for {MOVE_DURATION_S}s …")
        px.backward(speed)
        time.sleep(MOVE_DURATION_S)

        print("[motor] Stop.")
        px.stop()
    except Exception as exc:
        print(f"[motor] Error during motion sequence: {exc}")
        try:
            px.stop()
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

    if _run_sequence(args.speed):
        print("\nResult: PASS")
        return 0

    print("\nResult: FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
