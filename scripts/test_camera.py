# Hardware diagnostic script — direct SDK access intentional.
# Not a ROS node. Run on physical hardware only.

"""
Camera diagnostic script.

Captures a single frame from the CSI camera (Pi Camera via picamera2) and
saves it as a JPEG.

On Ubuntu 22.04, picamera2 requires libcamera Python bindings that must be
compiled from source. See "CSI Camera Setup" in README.md for full instructions.

Usage:
    python3 scripts/test_camera.py [--output PATH]

Exit codes:
    0  — image captured successfully
    1  — no working camera found
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional


DEFAULT_OUTPUT = Path("/tmp/camera_test.jpg")
DEV_LOG_DIR = "~/dev/autonomous-rover/log"
CAPTURE_TIMEOUT_S = 10

_KNOWN_SENSORS = ("ov5647", "imx219", "imx477", "imx708", "ov9281", "arducam")
_MEDIA_DEVICES = ("/dev/media0", "/dev/media1", "/dev/media2", "/dev/media3")


def _detect_csi_sensor() -> Optional[str]:
    """Return the sensor model string if a CSI camera is detected, else None."""
    for media_dev in _MEDIA_DEVICES:
        try:
            result = subprocess.run(
                ["media-ctl", "-d", media_dev, "--print-topology"],
                capture_output=True,
                text=True,
                timeout=5,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            continue

        for line in result.stdout.splitlines():
            low = line.lower()
            if "entity" in low and any(s in low for s in _KNOWN_SENSORS):
                return line.strip()

    return None


def _capture(output: Path) -> bool:
    """Capture a still frame via picamera2."""
    try:
        from picamera2 import Picamera2  # type: ignore
    except ImportError as exc:
        # Distinguish missing picamera2 wheel from missing libcamera bindings.
        if "libcamera" in str(exc):
            print(
                "[picamera2] libcamera Python bindings not found.\n"
                "           On Ubuntu 22.04 these are not in apt — see README.md."
            )
        else:
            print("[picamera2] Library not installed.")
        return False

    def _on_timeout(signum, frame):
        raise TimeoutError

    try:
        cam = Picamera2()
        config = cam.create_still_configuration(main={"size": (1280, 720)})
        cam.configure(config)
        cam.start()
        time.sleep(2)  # warm-up
        signal.signal(signal.SIGALRM, _on_timeout)
        signal.alarm(CAPTURE_TIMEOUT_S)
        cam.capture_file(str(output))
        signal.alarm(0)
        cam.stop()
        cam.close()
        print(f"[picamera2] Captured frame saved to: {output.resolve()}")
        return True
    except TimeoutError:
        print(f"[picamera2] Capture timed out after {CAPTURE_TIMEOUT_S}s — check camera cable.")
        return False
    except Exception as exc:
        print(f"[picamera2] Failed: {exc}")
        return False


def _scp_to_dev(local_path: Path) -> None:
    """Scp the captured image back to the dev machine if connected via SSH."""
    ssh_client_env = os.environ.get("SSH_CLIENT", "").split()
    if not ssh_client_env:
        print(f"Image saved locally: {local_path.resolve()}")
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
        print(f"Image copied to dev machine: {DEV_LOG_DIR}/{local_path.name}")
    else:
        print(f"scp failed: {result.stderr.strip()}")
        print(f"Image saved locally on Pi: {local_path.resolve()}")


def main() -> int:
    parser = argparse.ArgumentParser(description="CSI camera diagnostic")
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

    sensor = _detect_csi_sensor()
    if sensor:
        print(f"[media-ctl] CSI sensor detected: {sensor}")
    else:
        print("[media-ctl] No CSI sensor detected — check cable connection.")
    print()

    if _capture(output):
        print("\nResult: PASS")
        _scp_to_dev(output)
        return 0

    print("\nResult: FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
