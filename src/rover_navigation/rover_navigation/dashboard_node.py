"""
Live rover dashboard — runs on the Pi as part of nav.launch.py.

Subscribes to the camera, corrected depth, lidar, mode, and ultrasonic
topics and serves a combined view as MJPEG over HTTP for debugging — open
http://raspberrypi.local:8082 in a browser. Shows a 3x2 grid:
  - Top row: RGB camera feed, the corrected depth map annotated with the
    same per-region grid as scripts/depth_viewer.py, and ORB-SLAM3's own
    debug view (tracked keypoints on the current frame, green when tracking
    is OK and red otherwise — drawn by orb_slam3_node.cpp itself). These
    three are only populated under the camera/SLAM3 diagnostic launches
    (slam_preview.launch.py, slam_teleop.launch.py) — blank placeholders
    under nav.launch.py, which no longer runs the camera.
  - Bottom row: a live top-down scatter plot of ORB-SLAM3's accumulated map
    points (/orb_slam3/map_points), a live top-down scatter of the lidar's
    current /scan, and slam_toolbox's accumulated /map occupancy grid. The
    lidar panels are only populated under nav.launch.py (its primary
    localization/mapping source); the ORB-SLAM3 map-points panel is only
    populated under the camera diagnostic launches, same as the top row.
  - Current MANUAL/AUTO mode
  - Ultrasonic reading and the correction scale depth_bridge_node applied
  - The robot's estimated position, looked up from the map->base_link TF
    (map->odom is a static identity — there's no absolute correction source
    — and odom->base_link comes from slam_pose_bridge.py's ORB-SLAM3-derived
    pose, published on /rover/odom)

Unlike scripts/depth_viewer.py (a standalone diagnostic that calls the
depth server directly), this node only subscribes to already-published
topics — it shows exactly what the rest of the stack is actually using,
not a separate re-computation.
"""

import math
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud, Range
from std_msgs.msg import Float32, String
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

PORT = 8082
GRID_ROWS = 3
GRID_COLS = 4
STALE_AFTER_S = 2.0
MAX_MAP_POINTS = 20000
_ANSI_YELLOW = "\033[33m"
_ANSI_RESET = "\033[0m"


def _colorize_depth(depth: np.ndarray) -> np.ndarray:
    """Return a TURBO-colorized BGR image of the depth map, invalid pixels in black."""
    valid = depth[(depth > 0) & np.isfinite(depth)]
    lo = float(valid.min()) if valid.size else 0.0
    hi = float(np.percentile(valid, 98)) if valid.size else 1.0
    if hi <= lo:
        hi = lo + 1.0

    norm = np.clip((depth - lo) / (hi - lo), 0.0, 1.0)
    colored = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    colored[~((depth > 0) & np.isfinite(depth))] = 0
    return colored


def _draw_region_grid(panel: np.ndarray, depth: np.ndarray) -> None:
    """Overlay a grid on `panel`, labeling each cell with its mean depth in meters."""
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
            cv2.putText(
                panel,
                label,
                (tx, ty),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )


def _render_map_panel(points: list[tuple[float, float]], h: int, w: int) -> np.ndarray:
    """Top-down scatter plot of accumulated ORB-SLAM3 map points.

    `points` are (x, z) in the SLAM world frame (X-right, Z-forward — see
    slam_pose_bridge.py's coordinate-frame comment block), which is exactly
    a top-down view when the camera stays roughly level. Auto-scaled to fit
    every point seen so far, so the plot zooms out as the covered area grows.
    """
    panel = np.zeros((h, w, 3), dtype=np.uint8)
    if len(points) < 2:
        cv2.putText(
            panel,
            "no map points yet",
            (10, h // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return panel

    pts = np.array(points, dtype=np.float32)
    xs, zs = pts[:, 0], pts[:, 1]
    margin = 20
    x_range = max(float(xs.max() - xs.min()), 0.1)
    z_range = max(float(zs.max() - zs.min()), 0.1)
    scale = min((w - 2 * margin) / x_range, (h - 2 * margin) / z_range)

    px = ((xs - xs.min()) * scale + margin).astype(np.int32)
    # Flip vertically so "forward" (+Z) points up on screen.
    pz = (h - margin - (zs - zs.min()) * scale).astype(np.int32)
    np.clip(px, 0, w - 1, out=px)
    np.clip(pz, 0, h - 1, out=pz)
    panel[pz, px] = (0, 220, 255)

    ox = int((0.0 - xs.min()) * scale + margin)
    oz = int(h - margin - (0.0 - zs.min()) * scale)
    if 0 <= ox < w and 0 <= oz < h:
        cv2.drawMarker(panel, (ox, oz), (0, 0, 255), cv2.MARKER_CROSS, 10, 2)

    cv2.putText(
        panel,
        f"SLAM map (top-down, {len(points)} pts)",
        (6, 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return panel


def _render_scan_panel(scan: LaserScan | None, h: int, w: int) -> np.ndarray:
    """Live top-down scatter of the lidar's current /scan, centered on the lidar.

    Unlike _render_map_panel's auto-fit scaling (fine for a slowly-growing
    accumulated point set), this uses a fixed display radius so the view
    doesn't visibly jump/rescale every single frame — much easier to read
    live.
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

    display_radius_m = min(float(scan.range_max), 8.0)
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
        self._bridge = CvBridge()

        self._rgb: np.ndarray | None = None
        self._depth: np.ndarray | None = None
        self._slam_debug: np.ndarray | None = None
        self._slam_debug_stamp = 0.0
        self._map_points: list[tuple[float, float]] = []
        self._scan: LaserScan | None = None
        self._occupancy_grid: OccupancyGrid | None = None
        self._mode = "unknown"
        self._ultrasonic_range: float | None = None
        self._ultrasonic_stamp = 0.0
        self._scale: float | None = None
        self._scale_stamp = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            Image, "/rover/camera/image_raw", self._on_rgb, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/rover/camera/depth", self._on_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/orb_slam3/debug_image", self._on_slam_debug, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud, "/orb_slam3/map_points", self._on_map_points, qos_profile_sensor_data
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, "/map", self._on_occupancy_grid, qos_profile_sensor_data
        )
        self.create_subscription(String, "/rover/current_mode", self._on_mode, 10)
        self.create_subscription(
            Range, "/rover/ultrasonic/range", self._on_range, qos_profile_sensor_data
        )
        self.create_subscription(
            Float32,
            "/rover/camera/depth_correction_scale",
            self._on_scale,
            qos_profile_sensor_data,
        )

        self.create_timer(0.1, self._render)
        self.create_timer(2.0, self._log_pose)
        self.get_logger().info(f"Dashboard ready — open http://raspberrypi.local:{PORT}")

    def _on_rgb(self, msg: Image) -> None:
        self._rgb = self._bridge.imgmsg_to_cv2(msg, "bgr8")

    def _on_depth(self, msg: Image) -> None:
        self._depth = self._bridge.imgmsg_to_cv2(msg, "32FC1")

    def _on_slam_debug(self, msg: Image) -> None:
        self._slam_debug = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        self._slam_debug_stamp = time.monotonic()

    def _on_map_points(self, msg: PointCloud) -> None:
        remaining = MAX_MAP_POINTS - len(self._map_points)
        if remaining <= 0:
            return
        self._map_points.extend((p.x, p.z) for p in msg.points[:remaining])

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_occupancy_grid(self, msg: OccupancyGrid) -> None:
        self._occupancy_grid = msg

    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data

    def _on_range(self, msg: Range) -> None:
        self._ultrasonic_range = float(msg.range)
        self._ultrasonic_stamp = time.monotonic()

    def _on_scale(self, msg: Float32) -> None:
        self._scale = float(msg.data)
        self._scale_stamp = time.monotonic()

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
        if self._rgb is None:
            return
        h, w = self._rgb.shape[:2]
        rgb_panel = self._rgb.copy()

        if self._depth is not None:
            depth_panel = _colorize_depth(self._depth)
            _draw_region_grid(depth_panel, self._depth)
            if depth_panel.shape[:2] != (h, w):
                depth_panel = cv2.resize(depth_panel, (w, h))
        else:
            depth_panel = np.zeros((h, w, 3), dtype=np.uint8)
            cv2.putText(
                depth_panel,
                "no depth yet",
                (10, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        cv2.putText(
            rgb_panel,
            "RGB",
            (6, 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            depth_panel,
            "Depth (as published, ultrasonic-corrected)",
            (6, 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        now = time.monotonic()
        if self._slam_debug is not None and now - self._slam_debug_stamp < STALE_AFTER_S:
            slam_panel = self._slam_debug.copy()
            if slam_panel.shape[:2] != (h, w):
                slam_panel = cv2.resize(slam_panel, (w, h))
        else:
            slam_panel = np.zeros((h, w, 3), dtype=np.uint8)
            cv2.putText(
                slam_panel,
                "no SLAM debug image yet",
                (10, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        pose = self._lookup_pose()

        map_panel = _render_map_panel(self._map_points, h, w)
        scan_panel = _render_scan_panel(self._scan, h, w)
        occupancy_panel = _render_occupancy_panel(self._occupancy_grid, pose, h, w)
        top_row = np.hstack([rgb_panel, depth_panel, slam_panel])
        bottom_row = np.hstack([map_panel, scan_panel, occupancy_panel])
        combined = np.vstack([top_row, bottom_row])

        if self._ultrasonic_range is not None and now - self._ultrasonic_stamp < STALE_AFTER_S:
            us_text = f"ultrasonic: {self._ultrasonic_range:.2f}m"
            if self._scale is not None and now - self._scale_stamp < STALE_AFTER_S:
                us_text += f"   scale: {self._scale:.2f}"
        else:
            us_text = "ultrasonic: no reading"

        if pose is not None:
            pos_text = f"position: x={pose[0]:.2f}m  y={pose[1]:.2f}m  yaw={pose[2]:+.1f}°"
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
