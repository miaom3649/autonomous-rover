"""
Depth server live diagnostic.

Continuously grabs frames from /rover/camera/image_raw, POSTs them to the
Windows depth inference server, and streams the annotated result (RGB +
colorized depth, both overlaid with a grid of per-region mean distances) as
MJPEG over HTTP — open http://raspberrypi.local:8081 in a browser (same
pattern as scripts/camera_viewer.py).

Also subscribes to /rover/ultrasonic/range and applies the same scale
correction as depth_bridge_node.py (rescales the AI depth map so its
forward-center estimate matches the ultrasonic reading), so what you see
here matches what actually gets published for navigation. The current
ultrasonic reading and correction scale are overlaid on the image.

If nothing is already publishing the camera topic (e.g. nav.launch.py isn't
running), this script starts a standalone `camera_node` itself and stops it
again on exit — no need to launch the full stack just to test the depth
server. Pass --no-camera to disable this and fail fast instead.

Usage (on Pi):
    source /opt/ros/humble/setup.bash
    source ~/dev/autonomous-rover/install/setup.bash
    python3 scripts/depth_viewer.py [--url URL]
    # Ctrl+C to stop
"""
import argparse
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import requests
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Range

WATCH_PORT = 8081
GRID_ROWS = 3
GRID_COLS = 4

# Same defaults as depth_bridge_node.py's ultrasonic_correction parameters.
ULTRASONIC_MAX_AGE = 1.0
ULTRASONIC_REGION_FRAC = 0.2
CORRECTION_SCALE_MIN = 0.3
CORRECTION_SCALE_MAX = 3.0


def _camera_topic_has_publisher(topic: str) -> bool:
    """Check for an existing publisher on `topic` via the CLI (no rclpy node needed)."""
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", topic, "--verbose"],
            capture_output=True, text=True, timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    for line in result.stdout.splitlines():
        if line.strip().startswith("Publisher count:"):
            return int(line.split(":")[1].strip()) > 0
    return False


def _start_camera_node() -> subprocess.Popen:
    """Launch a standalone camera_node so this script can run without nav.launch.py."""
    print("[camera_node] no publisher on the image topic — starting camera_node standalone...")
    return subprocess.Popen(
        ["ros2", "run", "rover_camera", "camera_node"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


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
    return colored


def _draw_region_grid(panel: np.ndarray, depth: np.ndarray) -> None:
    """Overlay a grid on `panel`, labeling each cell with its mean depth in meters.

    `depth` and `panel` must have the same height/width. Modifies `panel` in place.
    """
    h, w = depth.shape
    cell_h, cell_w = h / GRID_ROWS, w / GRID_COLS

    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            y0, y1 = int(row * cell_h), int((row + 1) * cell_h)
            x0, x1 = int(col * cell_w), int((col + 1) * cell_w)
            cell = depth[y0:y1, x0:x1]
            valid = cell[(cell > 0) & np.isfinite(cell)]
            label = f"{valid.mean():.2f}m" if valid.size > cell.size * 0.1 else "N/A"

            cv2.rectangle(panel, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 255), 1)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
            tx = x0 + (x1 - x0 - tw) // 2
            ty = y0 + (y1 - y0 + th) // 2
            cv2.rectangle(panel, (tx - 3, ty - th - 3), (tx + tw + 3, ty + 4), (0, 0, 0), -1)
            cv2.putText(panel, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (0, 255, 255), 1, cv2.LINE_AA)


def _build_annotated_image(bgr: np.ndarray, depth: np.ndarray) -> np.ndarray:
    """RGB | colorized-depth side by side, both annotated with a shared per-region grid."""
    rgb_panel = cv2.resize(bgr, (depth.shape[1], depth.shape[0]))
    depth_panel = _colorize_depth(depth)

    _draw_region_grid(rgb_panel, depth)
    _draw_region_grid(depth_panel, depth)

    cv2.putText(rgb_panel, "RGB (measure by eye here)", (6, 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(depth_panel, "AI depth (region mean, meters)", (6, 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)

    return np.hstack([rgb_panel, depth_panel])


class _LiveFeed(Node):
    """Continuously updates `self.frame` and `self.ultrasonic_range` with the latest readings."""

    def __init__(self, topic: str) -> None:
        super().__init__("depth_server_watch")
        self._bridge = CvBridge()
        self.frame: np.ndarray | None = None
        self.ultrasonic_range: float | None = None
        self.ultrasonic_stamp: float = 0.0
        self.ultrasonic_min_range: float = 0.02
        self.ultrasonic_max_range: float = 4.0
        self.create_subscription(Image, topic, self._cb, qos_profile_sensor_data)
        self.create_subscription(
            Range, "/rover/ultrasonic/range", self._on_range, qos_profile_sensor_data
        )

    def _cb(self, msg: Image) -> None:
        self.frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")

    def _on_range(self, msg: Range) -> None:
        self.ultrasonic_range = float(msg.range)
        self.ultrasonic_stamp = time.monotonic()
        self.ultrasonic_min_range = float(msg.min_range)
        self.ultrasonic_max_range = float(msg.max_range)


def _apply_ultrasonic_correction(
    depth: np.ndarray, node: _LiveFeed
) -> tuple[np.ndarray, float | None]:
    """Rescale `depth` to match a fresh ultrasonic reading. See depth_bridge_node.py.

    Returns (possibly-corrected depth, scale factor used or None if unchanged).
    """
    us_range = node.ultrasonic_range
    us_age = time.monotonic() - node.ultrasonic_stamp
    if us_range is None or us_age > ULTRASONIC_MAX_AGE:
        return depth, None
    if us_range <= node.ultrasonic_min_range or us_range >= node.ultrasonic_max_range:
        return depth, None

    h, w = depth.shape
    half_h = max(1, int(h * ULTRASONIC_REGION_FRAC / 2))
    half_w = max(1, int(w * ULTRASONIC_REGION_FRAC / 2))
    cy, cx = h // 2, w // 2
    region = depth[cy - half_h:cy + half_h, cx - half_w:cx + half_w]
    valid = region[(region > 0) & np.isfinite(region)]
    if valid.size < region.size * 0.1:
        return depth, None

    ai_center_estimate = float(valid.mean())
    if ai_center_estimate <= 0:
        return depth, None

    scale = us_range / ai_center_estimate
    scale = min(max(scale, CORRECTION_SCALE_MIN), CORRECTION_SCALE_MAX)
    return (depth * scale).astype(np.float32), scale


class _MjpegHandler(BaseHTTPRequestHandler):
    latest_jpeg: bytes | None = None
    lock = threading.Lock()

    def log_message(self, *args) -> None:
        pass

    def do_GET(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _MjpegHandler.lock:
                    data = _MjpegHandler.latest_jpeg
                if data:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                time.sleep(0.05)
        except Exception:
            pass


def _infer_loop(node: _LiveFeed, url: str, timeout_s: float) -> None:
    while rclpy.ok():
        frame = node.frame
        if frame is None:
            time.sleep(0.1)
            continue
        try:
            depth, dt_ms = _post_and_decode(frame, url, timeout_s)
            depth, scale = _apply_ultrasonic_correction(depth, node)
            combined = _build_annotated_image(frame, depth)

            if scale is not None:
                us_line = f"ultrasonic: {node.ultrasonic_range:.2f}m   scale: {scale:.2f}"
            elif node.ultrasonic_range is not None:
                us_line = (f"ultrasonic: {node.ultrasonic_range:.2f}m   "
                           "(stale/out-of-range, no correction)")
            else:
                us_line = "ultrasonic: no reading (no correction)"
            cv2.putText(combined, us_line, (6, combined.shape[0] - 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(combined, f"round-trip: {dt_ms:.0f} ms", (6, combined.shape[0] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
            _, jpeg = cv2.imencode(".jpg", combined, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with _MjpegHandler.lock:
                _MjpegHandler.latest_jpeg = jpeg.tobytes()
        except Exception as exc:
            node.get_logger().warn(f"depth request failed: {exc}")
            time.sleep(1.0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Depth server live diagnostic")
    parser.add_argument("--url", default="http://192.168.1.151:8765/depth",
                        help="Depth server URL")
    parser.add_argument("--topic", default="/rover/camera/image_raw",
                        help="ROS2 image topic to grab from")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="HTTP request timeout in seconds")
    parser.add_argument("--no-camera", action="store_true",
                        help="Don't auto-start camera_node even if the topic has no publisher")
    args = parser.parse_args()

    print("=== Depth Server Live Diagnostic ===")
    print(f"URL   : {args.url}")
    print(f"Topic : {args.topic}")
    print()

    started_camera_proc = None
    if not args.no_camera and not _camera_topic_has_publisher(args.topic):
        started_camera_proc = _start_camera_node()

    try:
        rclpy.init()
        node = _LiveFeed(args.topic)
        threading.Thread(target=_infer_loop, args=(node, args.url, args.timeout),
                          daemon=True).start()

        server = HTTPServer(("0.0.0.0", WATCH_PORT), _MjpegHandler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(f"Live depth view ready — open http://raspberrypi.local:{WATCH_PORT}"
              "  (Ctrl+C to stop)")

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
            server.shutdown()
    finally:
        if started_camera_proc is not None:
            print("\nStopping the camera_node we started...")
            started_camera_proc.terminate()
            try:
                started_camera_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                started_camera_proc.kill()


if __name__ == "__main__":
    main()
