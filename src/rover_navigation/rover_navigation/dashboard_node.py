"""
Live rover dashboard — runs on the Pi as part of nav.launch.py.

Subscribes to the lidar, slam_toolbox map, mode, and ultrasonic topics and
serves a combined view as MJPEG over HTTP for debugging — open
http://raspberrypi.local:8082 in a browser. Shows a 1x2 grid:
  - Left: a live top-down scatter of the lidar's current /scan
  - Right: slam_toolbox's accumulated /map occupancy grid, with the robot's
    current position (from the map->base_link TF) marked on it, plus Nav2's
    current plan (/plan) drawn as a line with its endpoint (the goal) marked
    — populated either by a click-to-preview goal or an active navigation
  - Current MANUAL/AUTO mode
  - Ultrasonic reading
  - The robot's estimated position, looked up from the map->base_link TF
    (map->odom comes from slam_toolbox's scan matching; odom->base_link is
    a static identity — this rover has no wheel encoders)

The page also has interactive controls:
  - "Reset map": slam_toolbox has no built-in "clear the map and start over"
    service, so this works by killing the async_slam_toolbox_node process
    outright — nav.launch.py runs it with respawn=True, so launch
    immediately restarts it with a fresh, blank map.
  - Click the map panel to set a goal: fires a ComputePathToPose action
    (preview only, doesn't drive) — its result path is published to /plan
    as a side effect by planner_server regardless of who calls it, so the
    existing /plan-driven rendering above shows the previewed route and
    goal marker for free, with no separate preview state needed.
  - "Move": sends the last-set goal as a real NavigateToPose action goal,
    forcing mode to AUTO first (Nav2 commands are otherwise dropped by
    mode_controller_node in MANUAL mode).
  - "Clear goal": cancels any in-flight navigation and clears the local
    goal/path state.
  - "Toggle mode": flips MANUAL/AUTO via /rover/mode.
  - Directional pad: press-and-hold buttons publish /rover/cmd_vel_teleop
    (repeated every 200ms while held, under drive_node's 0.5s cmd_timeout)
    and force mode to MANUAL first.
"""

import json
import math
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
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
_INDEX_HTML = """<!doctype html>
<html>
<head>
<title>Rover Dashboard</title>
<style>
  body { margin:0; background:#111; color:#eee; font-family:sans-serif; user-select:none; }
  button { font-size:16px; padding:6px 14px; }
  #toolbar { padding:8px; display:flex; gap:8px; flex-wrap:wrap; align-items:center; }
  #hint { padding:0 8px 4px; font-size:13px; color:#999; }
  #dpad {
    padding:12px; display:grid; gap:4px; justify-content:center;
    grid-template-columns:56px 56px 56px; grid-template-rows:56px 56px 56px;
  }
  #dpad button { font-size:20px; padding:0; }
</style>
</head>
<body>
  <div id="toolbar">
    <button id="resetBtn">Reset map</button>
    <button id="moveBtn">Move</button>
    <button id="clearBtn">Clear goal</button>
    <button id="modeBtn">Toggle mode</button>
    <span id="status"></span>
  </div>
  <div id="hint">Click the map panel (right half) to set a goal</div>
  <img id="stream" src="/stream" style="width:100%;display:block">
  <div id="dpad">
    <div></div><button class="drive" data-dir="up">&#9650;</button><div></div>
    <button class="drive" data-dir="left">&#9664;</button><div></div>
    <button class="drive" data-dir="right">&#9654;</button>
    <div></div><button class="drive" data-dir="down">&#9660;</button><div></div>
  </div>
  <script>
    const status = document.getElementById('status');
    function flash(msg) {
      status.textContent = ' ' + msg;
      clearTimeout(flash._t);
      flash._t = setTimeout(() => { status.textContent = ''; }, 1500);
    }
    async function post(path, body) {
      try {
        const resp = await fetch(path, {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(body || {}),
        });
        return resp.ok;
      } catch (e) {
        return false;
      }
    }

    document.getElementById('resetBtn').onclick = async () => {
      flash((await post('/reset')) ? 'map reset' : 'failed');
    };
    document.getElementById('moveBtn').onclick = async () => {
      flash((await post('/goal/move')) ? 'moving...' : 'failed');
    };
    document.getElementById('clearBtn').onclick = async () => {
      flash((await post('/goal/clear')) ? 'goal cleared' : 'failed');
    };
    document.getElementById('modeBtn').onclick = async () => {
      flash((await post('/mode/toggle')) ? 'mode toggled' : 'failed');
    };

    // Right half of the combined stream is the map panel — click it to set a goal.
    document.getElementById('stream').addEventListener('click', async (e) => {
      const rect = e.target.getBoundingClientRect();
      const fx = (e.clientX - rect.left) / rect.width;
      const fy = (e.clientY - rect.top) / rect.height;
      if (fx < 0.5) {
        flash('click the map panel (right side) to set a goal');
        return;
      }
      const ok = await post('/goal/set', {fx: (fx - 0.5) * 2, fy: fy});
      flash(ok ? 'goal set' : 'failed');
    });

    // Press-and-hold d-pad: repeat the drive command while held (under
    // drive_node's 0.5s cmd_timeout), send one stop command on release.
    let driveTimer = null;
    function startDrive(dir) {
      stopDrive();
      post('/drive', {dir});
      driveTimer = setInterval(() => post('/drive', {dir}), 200);
    }
    function stopDrive() {
      if (driveTimer) { clearInterval(driveTimer); driveTimer = null; }
      post('/drive', {dir: 'stop'});
    }
    document.querySelectorAll('.drive').forEach((btn) => {
      const dir = btn.dataset.dir;
      btn.addEventListener('mousedown', () => startDrive(dir));
      btn.addEventListener('touchstart', (e) => { e.preventDefault(); startDrive(dir); });
      btn.addEventListener('mouseup', stopDrive);
      btn.addEventListener('mouseleave', stopDrive);
      btn.addEventListener('touchend', stopDrive);
      btn.addEventListener('touchcancel', stopDrive);
    });
  </script>
</body>
</html>
"""
# Matches mode_controller_node's publisher QoS — TRANSIENT_LOCAL so this
# (deliberately late-starting) node still gets the last published mode
# immediately on subscribing, instead of only future mode changes.
_MODE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
# Matches mode_controller_node's subscription QoS for /rover/mode and
# /rover/cmd_vel_teleop (plain RELIABLE + VOLATILE, its shared _RELIABLE profile).
_CMD_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
# Ackermann steering can't rotate in place, so left/right nudge a bit of
# forward speed along with the turn — otherwise the button would just steer
# the front wheels without visibly moving the rover.
_DRIVE_TWISTS = {
    "up": (0.2, 0.0),
    "down": (-0.2, 0.0),
    "left": (0.15, 1.0),
    "right": (0.15, -1.0),
    "stop": (0.0, 0.0),
}
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
    grid: OccupancyGrid | None,
    pose: tuple[float, float, float] | None,
    path: Path | None,
    h: int,
    w: int,
) -> np.ndarray:
    """Render slam_toolbox's accumulated /map (nav_msgs/OccupancyGrid), top-down.

    Also overlays Nav2's current global plan (/plan) as a line, with its final
    waypoint marked as the goal — /plan has no dedicated "current goal" topic,
    but the planner always ends its path at (or near) the requested goal.
    """
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

    # Grid row 0 is the origin (bottom in world coords) — flip once so +y (north) is up.
    img = cv2.flip(img, 0)
    if (gh, gw) != (h, w):
        img = cv2.resize(img, (w, h), interpolation=cv2.INTER_NEAREST)

    res = grid.info.resolution

    def to_display(x: float, y: float) -> tuple[float, float]:
        """World (map frame) -> display pixel coords, matching the flip+resize above.

        Overlays (path, goal marker, pose) are all drawn *after* the flip and
        resize, directly in display-pixel space with float precision kept
        until the final round() — computing them earlier in raw grid-cell
        units meant their size/thickness came from a fixed cell count, which
        is tiny on a big explored map and comically (and imprecisely, for
        the heading tick) oversized on a small fresh one, e.g. right after a
        map reset.
        """
        col = (x - grid.info.origin.position.x) / res
        row = (y - grid.info.origin.position.y) / res
        return col * (w / gw), (gh - 1 - row) * (h / gh)

    if path is not None and len(path.poses) >= 2:
        pts = np.array(
            [
                [round(v) for v in to_display(p.pose.position.x, p.pose.position.y)]
                for p in path.poses
            ],
            dtype=np.int32,
        )
        cv2.polylines(img, [pts], isClosed=False, color=(255, 0, 255), thickness=2)
        cv2.drawMarker(img, tuple(pts[-1]), (0, 255, 0), cv2.MARKER_TILTED_CROSS, 14, 2)

    if pose is not None:
        col = (pose[0] - grid.info.origin.position.x) / res
        row = (pose[1] - grid.info.origin.position.y) / res
        if 0 <= col < gw and 0 <= row < gh:
            # Dot + a short heading tick off its edge (power-button-icon
            # style) rather than a plain "+": this is a top-down world-frame
            # map (unlike the ego-centric scan panel), so heading isn't
            # always "up" or "right" — draw it explicitly instead of leaving
            # it to be guessed.
            dcol, drow = to_display(pose[0], pose[1])
            circle_r = 5.0
            tick_len = 20.0
            yaw_rad = math.radians(pose[2])
            dx, dy = math.cos(yaw_rad), -math.sin(yaw_rad)  # -sin: row axis is flipped
            center = (round(dcol), round(drow))
            tick_end = (
                round(dcol + (circle_r + tick_len) * dx),
                round(drow + (circle_r + tick_len) * dy),
            )
            cv2.circle(img, center, round(circle_r), (0, 0, 255), -1)
            cv2.line(img, center, tick_end, (0, 0, 255), 2)

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
    node: "DashboardNode | None" = None

    # path -> (node, parsed_json_body) -> None. Kept as simple lambdas since
    # each just forwards to one DashboardNode method.
    _ROUTES = {
        "/reset": lambda node, body: node.reset_map(),
        "/goal/set": lambda node, body: node.set_goal_from_fraction(
            float(body.get("fx", 0.5)), float(body.get("fy", 0.5))
        ),
        "/goal/move": lambda node, body: node.move_to_goal(),
        "/goal/clear": lambda node, body: node.clear_goal(),
        "/mode/toggle": lambda node, body: node.toggle_mode(),
        "/drive": lambda node, body: node.drive(str(body.get("dir", "stop"))),
    }

    def log_message(self, *args) -> None:
        pass

    def do_GET(self) -> None:
        if self.path == "/stream":
            self._serve_stream()
        else:
            self._serve_index()

    def _serve_index(self) -> None:
        body = _INDEX_HTML.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_stream(self) -> None:
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

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", 0) or 0)
        raw_body = self.rfile.read(length) if length else b""
        try:
            body = json.loads(raw_body) if raw_body else {}
        except json.JSONDecodeError:
            body = {}

        node = _MjpegHandler.node
        route = _MjpegHandler._ROUTES.get(self.path)
        if node is None or route is None:
            self.send_response(404)
            self.end_headers()
            return
        try:
            route(node, body)
        except Exception:
            node.get_logger().exception(f"dashboard POST {self.path} failed")
            self.send_response(500)
            self.end_headers()
            return

        resp_body = b"ok"
        self.send_response(200)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(resp_body)))
        self.end_headers()
        self.wfile.write(resp_body)


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("dashboard_node")

        self._scan: LaserScan | None = None
        self._occupancy_grid: OccupancyGrid | None = None
        self._nav_path: Path | None = None
        self._mode = "unknown"
        self._ultrasonic_range: float | None = None
        self._ultrasonic_stamp = 0.0
        self._goal_pose: tuple[float, float, float] | None = None
        self._nav_goal_handle = None

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
        self.create_subscription(Path, "/plan", self._on_plan, 10)

        self._mode_pub = self.create_publisher(String, "/rover/mode", _CMD_QOS)
        self._teleop_pub = self.create_publisher(Twist, "/rover/cmd_vel_teleop", _CMD_QOS)
        self._compute_path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self._navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_timer(0.1, self._render)
        self.create_timer(2.0, self._log_pose)
        self.get_logger().info(f"Dashboard ready — open http://raspberrypi.local:{PORT}")

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_occupancy_grid(self, msg: OccupancyGrid) -> None:
        self._occupancy_grid = msg

    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data

    def _on_plan(self, msg: Path) -> None:
        self._nav_path = msg

    def _on_range(self, msg: Range) -> None:
        self._ultrasonic_range = float(msg.range)
        self._ultrasonic_stamp = time.monotonic()

    def reset_map(self) -> None:
        """Kill slam_toolbox so launch (respawn=True) restarts it with a blank map."""
        self.get_logger().warn("Reset map requested — restarting slam_toolbox")
        self._occupancy_grid = None
        subprocess.run(["pkill", "-f", "async_slam_toolbox_node"], check=False)

    def set_goal_from_fraction(self, fx: float, fy: float) -> None:
        """Convert a click on the map panel (fractional x/y, 0..1) to a map-frame goal."""
        grid = self._occupancy_grid
        if grid is None:
            self.get_logger().warn("Can't set goal — no /map yet")
            return
        gw, gh = grid.info.width, grid.info.height
        res = grid.info.resolution
        col = fx * gw
        row = (1.0 - fy) * gh  # undo _render_occupancy_panel's vertical flip
        x = grid.info.origin.position.x + col * res
        y = grid.info.origin.position.y + row * res
        self._set_goal(x, y, 0.0)

    def _set_goal(self, x: float, y: float, yaw: float) -> None:
        self._goal_pose = (x, y, yaw)
        self.get_logger().info(f"Goal set: x={x:.2f} y={y:.2f} — previewing path")
        if not self._compute_path_client.server_is_ready():
            self.get_logger().warn("compute_path_to_pose server not ready")
            return
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self._pose_stamped(x, y, yaw)
        goal_msg.use_start = False
        future = self._compute_path_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_compute_path_response)

    def _on_compute_path_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("compute_path_to_pose goal rejected")
            return
        # No further action needed — planner_server publishes the resulting
        # path to /plan itself, which _on_plan/_render already pick up.
        goal_handle.get_result_async()

    def move_to_goal(self) -> None:
        """Send the last-set goal as a real NavigateToPose action, forcing AUTO mode."""
        if self._goal_pose is None:
            self.get_logger().warn("Move requested but no goal is set")
            return
        x, y, yaw = self._goal_pose
        self._publish_mode("AUTO")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._pose_stamped(x, y, yaw)
        future = self._navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_navigate_goal_response)

    def _on_navigate_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("navigate_to_pose goal rejected")
            return
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigate_result)

    def _on_navigate_result(self, future) -> None:
        self._nav_goal_handle = None
        self.get_logger().info(f"navigate_to_pose finished: status={future.result().status}")

    def clear_goal(self) -> None:
        """Cancel any in-flight navigation and clear the local goal/path state."""
        self._goal_pose = None
        self._nav_path = None
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

    def toggle_mode(self) -> None:
        self._publish_mode("MANUAL" if self._mode == "AUTO" else "AUTO")

    def drive(self, direction: str) -> None:
        linear, angular = _DRIVE_TWISTS.get(direction, (0.0, 0.0))
        self._publish_mode("MANUAL")
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._teleop_pub.publish(msg)

    def _publish_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)

    def _pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

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
        occupancy_panel = _render_occupancy_panel(
            self._occupancy_grid, pose, self._nav_path, PANEL_H, PANEL_W
        )
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
    _MjpegHandler.node = node

    server = ThreadingHTTPServer(("0.0.0.0", PORT), _MjpegHandler)
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
