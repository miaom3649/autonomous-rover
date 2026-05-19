# Hardware diagnostic script — direct SDK access intentional.
# Not a ROS node. Run on physical hardware only.

"""
Steering servo diagnostic script.

Sweeps the front-wheel steering servo: center → left → center → right → center.
Validates that the servo responds and turns smoothly across its range.

Run with --calibrate to enter interactive trim mode: press +/- to nudge the
servo until the wheel is visually centered, then press Enter to save the offset.
Saved trim is applied automatically on every subsequent run.

Usage:
    python3 scripts/test_servo.py [--angle ANGLE] [--trim OFFSET]
    python3 scripts/test_servo.py --calibrate

Exit codes:
    0  — sweep completed (or calibration saved) without errors
    1  — Robot Hat not detected, picarx SDK not available, or servo error
"""

import argparse
import json
import sys
import time
from pathlib import Path


DEFAULT_ANGLE = 30      # degrees, max deflection from center
HOLD_S = 2              # seconds to hold each position
TRIM_FILE = Path("/opt/picar-x/servo_trim.json")

# Robot Hat V4 sits at one of these I2C addresses on bus 1.
_ROBOT_HAT_I2C_ADDRS = (0x14, 0x15)
_I2C_BUS = 1


def _load_trim() -> int:
    """Return saved trim offset in degrees, or 0 if none saved."""
    try:
        return int(json.loads(TRIM_FILE.read_text()).get("trim", 0))
    except Exception:
        return 0


def _save_trim(offset: int) -> None:
    TRIM_FILE.parent.mkdir(parents=True, exist_ok=True)
    TRIM_FILE.write_text(json.dumps({"trim": offset}))


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


def _init_picarx():
    """Return a Picarx instance or None on failure."""
    try:
        from picarx import Picarx  # type: ignore
    except ImportError:
        print("[picarx] Library not installed — run on the Raspberry Pi with the Sunfounder SDK.")
        return None
    try:
        return Picarx()
    except Exception as exc:
        print(f"[picarx] Failed to initialise: {exc}")
        return None


def _run_calibrate() -> bool:
    """Interactive trim mode: nudge servo with +/- until centered, Enter to save."""
    try:
        import readchar  # type: ignore
    except ImportError:
        print("[calibrate] readchar not installed — run: pip3 install readchar")
        return False

    px = _init_picarx()
    if px is None:
        return False

    offset = _load_trim()
    print(f"Current trim: {offset:+d}°")
    print("Controls: '+' nudge right  '-' nudge left  Enter save & exit  Ctrl-C cancel")
    print()

    try:
        while True:
            px.set_dir_servo_angle(offset)
            print(f"\r  trim = {offset:+3d}°   ", end="", flush=True)
            key = readchar.readkey()
            if key in ("+", "="):
                offset += 1
            elif key == "-":
                offset -= 1
            elif key in (readchar.key.ENTER, "\r", "\n"):
                break
    except KeyboardInterrupt:
        print("\nCalibration cancelled.")
        px.set_dir_servo_angle(0)
        return False
    finally:
        px.set_dir_servo_angle(offset)

    _save_trim(offset)
    print(f"\nTrim {offset:+d}° saved to {TRIM_FILE}")
    return True


def _run_sweep(angle: int, trim: int) -> bool:
    """Sweep steering servo through center/left/right applying trim offset."""
    px = _init_picarx()
    if px is None:
        return False

    if trim:
        print(f"[servo] Applying trim offset: {trim:+d}°")

    steps = [
        (trim,          "Center"),
        (trim - angle,  "Left"),
        (trim,          "Center"),
        (trim + angle,  "Right"),
        (trim,          "Center"),
    ]

    try:
        for deg, label in steps:
            print(f"[servo] {label} ({deg:+d}°) — holding {HOLD_S}s …")
            px.set_dir_servo_angle(deg)
            time.sleep(HOLD_S)
    except Exception as exc:
        print(f"[servo] Error during sweep: {exc}")
        try:
            px.set_dir_servo_angle(trim)
        except Exception:
            pass
        return False
    finally:
        px.set_dir_servo_angle(trim)

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
    parser.add_argument(
        "--trim",
        type=int,
        default=None,
        metavar="OFFSET",
        help="Override saved trim offset (degrees)",
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Enter interactive trim calibration mode",
    )
    args = parser.parse_args()

    if not _detect_robot_hat():
        print("\nResult: FAIL")
        return 1

    if args.calibrate:
        print("=== Servo Calibration ===")
        print()
        ok = _run_calibrate()
        print("\nResult:", "PASS" if ok else "FAIL")
        return 0 if ok else 1

    if not 0 < args.angle <= 45:
        print(f"Error: --angle must be between 1 and 45, got {args.angle}")
        return 1

    trim = args.trim if args.trim is not None else _load_trim()

    print("=== Servo Diagnostic ===")
    print(f"Max angle   : ±{args.angle}°")
    print(f"Trim offset : {trim:+d}°")
    print(f"Hold time   : {HOLD_S}s per position")
    print()

    if _run_sweep(args.angle, trim):
        print("\nResult: PASS")
        return 0

    print("\nResult: FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
