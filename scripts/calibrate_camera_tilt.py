# Hardware diagnostic script — direct SDK access intentional.
# Not a ROS node. Run on physical hardware only.
"""
Camera tilt servo calibration script.

Run on the Pi:
    python3 scripts/calibrate_camera_tilt.py

The script steps through tilt angles and waits for you to press Enter
after visually confirming whether the camera is level. When done, it
prints the angle to put in drive_node.py.
"""

from picarx import Picarx  # type: ignore[import]


def main() -> None:
    px = Picarx()
    px.set_cam_pan_angle(0)

    angles = [-40, -30, -20, -15, -10, -5, 0, 5, 10, 15, 20, 30, 40]
    chosen: int | None = None

    print("Camera tilt calibration. Look at the camera and press Enter at each step.")
    print("Type 'y' + Enter when the camera looks level and forward-facing.\n")

    for angle in angles:
        px.set_cam_tilt_angle(angle)
        answer = input(f"  angle={angle:+d}°  — level? [y/Enter to skip]: ").strip().lower()
        if answer == "y":
            chosen = angle
            break

    if chosen is not None:
        print(f"\nCalibrated tilt angle: {chosen}")
        print("Update drive_node.py:")
        print(f"    self._px.set_cam_tilt_angle({chosen})")
    else:
        print("\nNo angle selected. Try a finer range or check servo mechanically.")

    px.set_cam_tilt_angle(0)


if __name__ == "__main__":
    main()
