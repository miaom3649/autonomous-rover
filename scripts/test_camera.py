# Hardware diagnostic script — direct SDK access intentional.
# Not a ROS node. Run on physical hardware only.

"""
Camera diagnostic script.

Attempts to capture a single frame from the camera and save it as a JPEG.
Tries Pi Camera (picamera2) first; falls back to USB camera (OpenCV) if
picamera2 is unavailable or no CSI camera is detected.

Usage:
    python3 scripts/test_camera.py [--output PATH]

Exit codes:
    0  — image captured successfully
    1  — no working camera found
"""

import argparse
import sys
import time
from pathlib import Path


CAPTURE_TIMEOUT_S = 5
DEFAULT_OUTPUT = Path("camera_test.jpg")


def try_picamera2(output: Path) -> bool:
    """Attempt capture via picamera2 (CSI / Pi Camera)."""
    try:
        from picamera2 import Picamera2  # type: ignore
    except ImportError:
        print("[picamera2] Library not installed — skipping CSI camera.")
        return False

    try:
        cam = Picamera2()
        config = cam.create_still_configuration(main={"size": (1280, 720)})
        cam.configure(config)
        cam.start()
        time.sleep(2)  # warm-up
        cam.capture_file(str(output))
        cam.stop()
        cam.close()
        print(f"[picamera2] Captured frame saved to: {output.resolve()}")
        return True
    except Exception as exc:
        print(f"[picamera2] Failed: {exc}")
        return False


def try_opencv(output: Path) -> bool:
    """Attempt capture via OpenCV (USB / V4L2 camera)."""
    try:
        import cv2  # type: ignore
    except ImportError:
        print("[opencv]   Library not installed — skipping USB camera.")
        return False

    for device_index in range(4):
        cap = cv2.VideoCapture(device_index)
        if not cap.isOpened():
            continue

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        ret, frame = cap.read()
        cap.release()

        if ret and frame is not None:
            cv2.imwrite(str(output), frame)
            print(
                f"[opencv]   Captured frame from /dev/video{device_index} "
                f"saved to: {output.resolve()}"
            )
            return True
        else:
            print(f"[opencv]   /dev/video{device_index} opened but read failed.")

    print("[opencv]   No readable USB camera found on /dev/video0–3.")
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Camera hardware diagnostic")
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"Output JPEG path (default: {DEFAULT_OUTPUT})",
    )
    args = parser.parse_args()

    output: Path = args.output

    print("=== Camera Diagnostic ===")
    print(f"Output path : {output.resolve()}")
    print()

    if try_picamera2(output):
        print("\nResult: PASS (CSI / Pi Camera)")
        return 0

    print()

    if try_opencv(output):
        print("\nResult: PASS (USB Camera via OpenCV)")
        return 0

    print("\nResult: FAIL — no working camera detected.")
    print("Check connections and ensure at least one of the following is installed:")
    print("  CSI camera : pip install picamera2")
    print("  USB camera : pip install opencv-python")
    return 1


if __name__ == "__main__":
    sys.exit(main())
