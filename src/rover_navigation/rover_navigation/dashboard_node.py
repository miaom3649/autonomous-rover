"""
Live rover dashboard — runs on the Pi as part of nav.launch.py.

Subscribes to the lidar, slam_toolbox map, mode, and ultrasonic topics and
serves a combined view as MJPEG over HTTP for debugging — open
http://raspberrypi.local:8082 in a browser. Shows a 1x2 grid:
  - Left: a live top-down scatter of the lidar's current /scan
  - Right: slam_toolbox's accumulated /map occupancy grid, with the robot's
    current position (from the map->base_link TF) marked on it
  - Current MANUAL/AUTO mode
  - Ultrasonic reading
  - The robot's estimated position, looked up from the map->base_link TF
    (map->odom comes from slam_toolbox's scan matching; odom->base_link is
    a static identity — this rover has no wheel encoders)
"""

import math
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

PORT = 8082
STALE_AFTER_S = 2.0
PANEL_H = 480
PANEL_W = 640
SCAN_DISPLAY_RADIUS_M = 4.0
# Matches mode_controller_node's publisher QoS — TRANSIENT_LOCAL so this
# (deliberately late-starting) node still gets the last published mode
# immediately on subscribing, instead of only future mode changes.
_MODE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
_ANSI_YELLOW = "\033[33m"
_ANSI_RESET = "\033[0m"


def _render_scan_panel(scan: LaserScan | None, h: int, w: int) -> np.ndarray:
    """Live top-down scatter of the lidar's current /scan, centered on the lidar.

    Uses a fixed display radius (rather than auto-fitting to the data, like
    the occupancy panel's map) so the view doesn't visibly jump/rescale every
    single frame — much easier to read live.
    """
    panel = np.zeros((h, w, 3), dtype=np.uint8)
    if scan is None or not scan.ranges:
        cv2.putText(
            panel,
            "no /scan yet",
            (10, h // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return panel

    ranges = np.array(scan.ranges, dtype=np.float32)
    angles = scan.angle_min + np.arange(len(ranges), dtype=np.float32) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    ranges, angles = ranges[valid], angles[valid]

    display_radius_m = min(float(scan.range_max), SCAN_DISPLAY_RADIUS_M)
    margin = 20
    scale = (min(h, w) / 2 - margin) / display_radius_m
    cx, cy = w // 2, h // 2

    xs = cx + ranges * np.cos(angles) * scale
    ys = cy - ranges * np.sin(angles) * scale  # flip so +y (left) is up on screen
    px = np.clip(xs.astype(np.int32), 0, w - 1)
    py = np.clip(ys.astype(np.int32), 0, h - 1)
    panel[py, px] = (0, 220, 255)

    cv2.drawMarker(panel, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
    cv2.putText(
        panel,
        f"lidar /scan (live, {display_radius_m:.0f}m radius, {valid.sum()} pts)",
        (6, 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return panel


def _render_occupancy_panel(
    grid: OccupancyGrid | None, pose: tuple[float, float, float] | None, h: int, w: int
) -> np.ndarray:
    """Render slam_toolbox's accumulated /map (nav_msgs/OccupancyGrid), top-down."""
    if grid is None or grid.info.width == 0 or grid.info.height == 0:
        panel = np.zeros((h, w, 3), dtype=np.uint8)
        cv2.putText(
            panel,
            "no /map yet",
            (10, h // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return panel

    gw, gh = grid.info.width, grid.info.height
    cells = np.array(grid.data, dtype=np.int16).reshape(gh, gw)
    gray = np.where(cells < 0, 127, 255 - (np.clip(cells, 0, 100) * 255 // 100)).astype(np.uint8)
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    if pose is not None:
        res = grid.info.resolution
        col = (pose[0] - grid.info.origin.position.x) / res
        row = (pose[1] - grid.info.origin.position.y) / res
        px, py = int(col), int(row)
        if 0 <= px < gw and 0 <= py < gh:
            cv2.drawMarker(img, (px, py), (0, 0, 255), cv2.MARKER_CROSS, max(gw // 40, 4), 2)

    # Grid row 0 is the origin (bottom in world coords) — flip once so +y (north) is up.
    img = cv2.flip(img, 0)
    if (gh, gw) != (h, w):
        img = cv2.resize(img, (w, h), interpolation=cv2.INTER_NEAREST)
    cv2.putText(
        img,
        f"slam_toolbox map ({gw}x{gh} @ {grid.info.resolution:.2f}m/px)",
        (6, 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (0, 255, 0),
        1,
        cv2.LINE_AA,
    )
    return img


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


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("dashboard_node")

        self._scan: LaserScan | None = None
        self._occupancy_grid: OccupancyGrid | None = None
        self._mode = "unknown"
        self._ultrasonic_range: float | None = None
        self._ultrasonic_stamp = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, "/map", self._on_occupancy_grid, qos_profile_sensor_data
        )
        self.create_subscription(String, "/rover/current_mode", self._on_mode, _MODE_QOS)
        self.create_subscription(
            Range, "/rover/ultrasonic/range", self._on_range, qos_profile_sensor_data
        )

        self.create_timer(0.1, self._render)
        self.create_timer(2.0, self._log_pose)
        self.get_logger().info(f"Dashboard ready — open http://raspberrypi.local:{PORT}")

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_occupancy_grid(self, msg: OccupancyGrid) -> None:
        self._occupancy_grid = msg

    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data

    def _on_range(self, msg: Range) -> None:
        self._ultrasonic_range = float(msg.range)
        self._ultrasonic_stamp = time.monotonic()

    def _lookup_pose(self) -> tuple[float, float, float] | None:
        """Return (x, y, yaw_deg) from the map->base_link TF, or None if unavailable."""
        try:
            t = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, math.degrees(yaw))

    def _log_pose(self) -> None:
        pose = self._lookup_pose()
        if pose is None:
            self.get_logger().info(
                f"{_ANSI_YELLOW}pose  no map->base_link transform yet{_ANSI_RESET}"
            )
            return
        x, y, yaw_deg = pose
        self.get_logger().info(
            f"{_ANSI_YELLOW}pose  x={x:+.3f}  y={y:+.3f}  yaw={yaw_deg:+.1f}°{_ANSI_RESET}"
        )

    def _render(self) -> None:
        pose = self._lookup_pose()

        scan_panel = _render_scan_panel(self._scan, PANEL_H, PANEL_W)
        occupancy_panel = _render_occupancy_panel(self._occupancy_grid, pose, PANEL_H, PANEL_W)
        combined = np.hstack([scan_panel, occupancy_panel])

        now = time.monotonic()
        if self._ultrasonic_range is not None and now - self._ultrasonic_stamp < STALE_AFTER_S:
            us_text = f"ultrasonic: {self._ultrasonic_range:.2f}m"
        else:
            us_text = "ultrasonic: no reading"

        if pose is not None:
            # cv2's Hershey font has no glyph for '°' — spell it out instead.
            pos_text = f"position: x={pose[0]:.2f}m  y={pose[1]:.2f}m  yaw={pose[2]:+.1f}deg"
        else:
            pos_text = "position: no map->base_link transform yet"

        mode_text = f"mode: {self._mode}"

        for i, text in enumerate((mode_text, us_text, pos_text)):
            y = combined.shape[0] - 8 - (2 - i) * 18
            cv2.putText(
                combined,
                text,
                (6, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

        _, jpeg = cv2.imencode(".jpg", combined, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with _MjpegHandler.lock:
            _MjpegHandler.latest_jpeg = jpeg.tobytes()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DashboardNode()

    server = HTTPServer(("0.0.0.0", PORT), _MjpegHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        server.shutdown()


if __name__ == "__main__":
    main()
