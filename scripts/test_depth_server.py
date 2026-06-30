"""
Depth server end-to-end diagnostic.

Grabs one frame from /rover/camera/image_raw (requires camera_node to be
running), POSTs it to the Windows depth inference server, prints depth
statistics, and saves a side-by-side RGB + colorized depth image.

Usage (on Pi, with nav.launch.py or camera_node already running):
    source /opt/ros/humble/setup.bash
    source ~/dev/autonomous-rover/install/setup.bash
    python3 scripts/test_depth_server.py [--url URL]

Outputs saved to log/ and scped to the dev machine if running over SSH:
    <ts>_rgb.jpg        captured RGB frame sent to the server
    <ts>_depth.png      side-by-side RGB | colorized depth (TURBO, meters)
    <ts>_depth_raw.npy  raw float32 depth array in meters
"""
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import requests
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

DEV_LOG_DIR = "~/dev/autonomous-rover/log"


class _OneShot(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("depth_server_probe")
        self._bridge = CvBridge()
        self._frame: np.ndarray | None = None
        self.create_subscription(Image, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg: Image) -> None:
        if self._frame is None:
            self._frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info(
                f"Frame grabbed: {self._frame.shape[1]}×{self._frame.shape[0]}"
            )


def _grab_frame(topic: str, timeout_s: float = 10.0) -> np.ndarray:
    """Subscribe once to a camera topic and return the first frame as BGR."""
    rclpy.init()
    node = _OneShot(topic)
    deadline = time.monotonic() + timeout_s
    while node._frame is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    frame = node._frame
    node.destroy_node()
    rclpy.shutdown()
    if frame is None:
        print(f"ERROR: no message on {topic} within {timeout_s}s — is camera_node running?")
        sys.exit(1)
    return frame


def _post_and_decode(bgr: np.ndarray, url: str, timeout_s: float) -> tuple[np.ndarray, float]:
    """Encode frame as JPEG, POST to depth server, return (depth_f32, round_trip_ms)."""
    _, enc = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
    t0 = time.perf_counter()
    resp = requests.post(
        url,
        data=bytes(enc),
        headers={"Content-Type": "image/jpeg"},
        timeout=timeout_s,
    )
    dt_ms = (time.perf_counter() - t0) * 1000
    resp.raise_for_status()
    h = int(resp.headers["X-Depth-Height"])
    w = int(resp.headers["X-Depth-Width"])
    depth = np.frombuffer(resp.content, dtype=np.float32).reshape(h, w)
    return depth, dt_ms


def _colorize_depth(depth: np.ndarray) -> np.ndarray:
    """Return a TURBO-colorized BGR image of the depth map, invalid pixels in black."""
    valid = depth[(depth > 0) & np.isfinite(depth)]
    lo = float(valid.min()) if valid.size else 0.0
    # clip at 98th-percentile so a few outlier far points don't wash out nearby detail
    hi = float(np.percentile(valid, 98)) if valid.size else 1.0
    if hi <= lo:
        hi = lo + 1.0

    norm = np.clip((depth - lo) / (hi - lo), 0.0, 1.0)
    colored = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    colored[~((depth > 0) & np.isfinite(depth))] = 0  # black out invalid pixels

    stats = (
        f"min={valid.min():.2f}m  max={valid.max():.2f}m  "
        f"mean={valid.mean():.2f}m  valid={100 * valid.size / depth.size:.0f}%"
        if valid.size
        else "ALL INVALID — depth values are 0 or non-finite"
    )
    cv2.putText(
        colored, stats, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA
    )
    return colored


def _scp_files(paths: list[Path]) -> None:
    ssh_env = os.environ.get("SSH_CLIENT", "").split()
    if not ssh_env:
        print("Not running over SSH — files saved locally on Pi:")
        for p in paths:
            print(f"  {p.resolve()}")
        return
    dev_ip = ssh_env[0]
    dev_user = os.environ.get("ROVER_DEV_USER", "konkon")
    remote = f"{dev_user}@{dev_ip}:{DEV_LOG_DIR}/"
    for path in paths:
        r = subprocess.run(["scp", str(path), remote], capture_output=True, text=True)
        if r.returncode == 0:
            print(f"  → {DEV_LOG_DIR}/{path.name}")
        else:
            print(f"  scp failed for {path.name}: {r.stderr.strip()}")
            print(f"  (saved locally: {path.resolve()})")


def main() -> None:
    parser = argparse.ArgumentParser(description="Depth server end-to-end diagnostic")
    parser.add_argument("--url", default="http://192.168.1.151:8765/depth",
                        help="Depth server URL")
    parser.add_argument("--topic", default="/rover/camera/image_raw",
                        help="ROS2 image topic to grab from")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="HTTP request timeout in seconds")
    args = parser.parse_args()

    log_dir = Path(__file__).resolve().parent.parent / "log"
    log_dir.mkdir(exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")

    print("=== Depth Server Diagnostic ===")
    print(f"URL   : {args.url}")
    print(f"Topic : {args.topic}")
    print()

    # ── Step 1: grab a camera frame from the running ROS camera node ──────────
    print("Step 1: grabbing frame from ROS topic...")
    bgr = _grab_frame(args.topic, timeout_s=10.0)
    rgb_path = log_dir / f"{ts}_rgb.jpg"
    cv2.imwrite(str(rgb_path), bgr)
    print(f"  saved: {rgb_path.name}")

    # ── Step 2: call depth server ─────────────────────────────────────────────
    print(f"\nStep 2: POSTing JPEG to {args.url} ...")
    try:
        depth, dt_ms = _post_and_decode(bgr, args.url, args.timeout)
    except requests.Timeout:
        print(f"  ERROR: timeout ({args.timeout}s) — is depth_server.py running on Windows?")
        sys.exit(1)
    except Exception as exc:
        print(f"  ERROR: {exc}")
        sys.exit(1)
    print(f"  round-trip: {dt_ms:.0f} ms   depth shape: {depth.shape}   dtype: {depth.dtype}")

    # ── Step 3: print statistics ──────────────────────────────────────────────
    print("\nStep 3: depth statistics")
    valid = depth[(depth > 0) & np.isfinite(depth)]
    print(f"  raw values:   min={depth.min():.4f}  max={depth.max():.4f}  mean={depth.mean():.4f}")
    print(f"  valid pixels: {valid.size} / {depth.size}  ({100 * valid.size / depth.size:.1f}%)")
    if valid.size:
        pcts = np.percentile(valid, [5, 25, 50, 75, 95])
        print(f"  percentiles (5/25/50/75/95): "
              f"{pcts[0]:.2f}  {pcts[1]:.2f}  {pcts[2]:.2f}  {pcts[3]:.2f}  {pcts[4]:.2f} m")
    else:
        print("  *** WARNING: no valid depth — check depth_server.py output ***")

    # ── Step 4: save visualizations ───────────────────────────────────────────
    print("\nStep 4: saving visualizations...")
    colored = _colorize_depth(depth)
    rgb_resized = cv2.resize(bgr, (depth.shape[1], depth.shape[0]))
    combined = np.hstack([rgb_resized, colored])

    depth_vis_path = log_dir / f"{ts}_depth.png"
    npy_path = log_dir / f"{ts}_depth_raw.npy"
    cv2.imwrite(str(depth_vis_path), combined)
    np.save(str(npy_path), depth)
    print(f"  {depth_vis_path.name}  (left = RGB input, right = depth TURBO colormap)")
    print(f"  {npy_path.name}         (raw float32 meters, load with np.load)")

    # ── Step 5: scp results to dev machine ───────────────────────────────────
    print("\nStep 5: transferring to dev machine...")
    _scp_files([rgb_path, depth_vis_path, npy_path])

    print("\n=== DONE ===")


if __name__ == "__main__":
    main()
