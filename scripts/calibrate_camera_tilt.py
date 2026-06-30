# Hardware diagnostic script — direct SDK access intentional.
# Not a ROS node. Run on physical hardware only.
"""
Camera servo calibration — interactive arrow-key control.

Run on the Pi:
    python3 scripts/calibrate_camera_tilt.py

  ↑ / ↓  — tilt up / down
  ← / →  — pan left / right
  Enter   — confirm and print values for drive_node.py
  q       — quit without saving
"""

import sys
import termios
import tty

from picarx import Picarx  # type: ignore[import]

STEP = 1  # degrees per key press
LIMIT = 45


def read_key() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            return f"{ch}{ch2}{ch3}"
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def clamp(val: int) -> int:
    return max(-LIMIT, min(LIMIT, val))


def main() -> None:
    px = Picarx()
    tilt = 0
    pan = 0
    px.set_cam_tilt_angle(tilt)
    px.set_cam_pan_angle(pan)

    print("Camera calibration — use arrow keys to adjust, Enter to save, q to quit.")
    print(f"  tilt={tilt:+d}°  pan={pan:+d}°  (limit ±{LIMIT}°)")

    while True:
        key = read_key()

        if key == "\x1b[A":    # up
            tilt = clamp(tilt + STEP)
            px.set_cam_tilt_angle(tilt)
        elif key == "\x1b[B":  # down
            tilt = clamp(tilt - STEP)
            px.set_cam_tilt_angle(tilt)
        elif key == "\x1b[D":  # left
            pan = clamp(pan - STEP)
            px.set_cam_pan_angle(pan)
        elif key == "\x1b[C":  # right
            pan = clamp(pan + STEP)
            px.set_cam_pan_angle(pan)
        elif key in ("\r", "\n"):
            break
        elif key in ("q", "\x03"):
            print("\nAborted.")
            px.set_cam_tilt_angle(0)
            px.set_cam_pan_angle(0)
            return

        print(f"\r  tilt={tilt:+d}°  pan={pan:+d}°        ", end="", flush=True)

    print(f"\n\nCalibrated values:")
    print(f"    self._px.set_cam_tilt_angle({tilt})")
    print(f"    self._px.set_cam_pan_angle({pan})")


if __name__ == "__main__":
    main()
