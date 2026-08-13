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
  - "Reset map": asks mapping_monitor_node to replace the current
    slam_toolbox process with a fresh mapping instance.
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
  - Status bar (top): current navigation status (not started / planning
    failed / navigating / succeeded / canceled / failed) plus a live,
    independent "obstacle blocking forward motion" indicator sourced from
    mode_controller_node's /rover/obstacle_blocked — polled from /nav_status
    every 500ms.
"""

import json
import math
from pathlib import Path as FilePath
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
from sensor_msgs.msg import Image, LaserScan, Range
from std_msgs.msg import Bool, String
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

from rover_navigation.ground_projection import pixel_to_ground

PORT = 8082
STALE_AFTER_S = 2.0
# action_msgs/msg/GoalStatus terminal values
_GOAL_STATUS_TEXT = {4: "Goal reached", 5: "Canceled", 6: "Navigation failed"}
PANEL_H = 480
PANEL_W = 640
SCAN_DISPLAY_RADIUS_M = 4.0
_INDEX_HTML = """<!doctype html>
<html>
<head>
<title>Rover Dashboard</title>
<style>
  * { box-sizing:border-box; }
  html, body { margin:0; width:100%; height:100%; background:#111; color:#eee;
               font-family:sans-serif; user-select:none; overflow:hidden; }
  body { display:grid; grid-template-rows:auto auto auto minmax(0, 1fr); }
  button, input { font-size:clamp(11px, 1.1vw, 15px); padding:5px 9px; max-width:100%; }
  #toolbar { padding:5px; display:flex; gap:4px; flex-wrap:wrap; align-items:center;
             width:100%; }
  #hint { padding:2px 5px; font-size:clamp(10px, 1vw, 13px); color:#999; }
  #content { min-height:0; width:100%; display:grid; grid-template-columns:minmax(0, 2fr) minmax(240px, 1fr); gap:5px; padding:0 5px 5px; }
  #mapSection, #cameraSection { min-width:0; min-height:0; display:grid; overflow:hidden; }
  #mapSection { grid-template-rows:auto minmax(0, 1fr); }
  #cameraSection { grid-template-rows:auto minmax(0, 1fr) auto; border-left:1px solid #333; }
  #driveArea { display:flex; align-items:center; justify-content:center; gap:10px; min-height:0; }
  #dpad {
    padding:4px; display:grid; gap:3px; justify-content:center;
    grid-template-columns:clamp(34px, 4vw, 48px) clamp(34px, 4vw, 48px) clamp(34px, 4vw, 48px);
    grid-template-rows:clamp(34px, 4vw, 48px) clamp(34px, 4vw, 48px) clamp(34px, 4vw, 48px);
  }
  #dpad button { font-size:clamp(14px, 1.5vw, 19px); padding:0; }
  #navStatus {
    padding:6px 12px; font-size:14px; background:#333; border-bottom:1px solid #000;
  }
  #navStatus.obstacle { background:#5a1a1a; color:#ffb3b3; }
  #mapWrap { position:relative; min-width:0; min-height:0; overflow:hidden; place-self:center; background:#000; }
  #stream { width:100%; height:100%; object-fit:contain; display:block; background:#000; }
  #traceCanvas { position:absolute; inset:0; width:100%; height:100%; pointer-events:auto; }
  #tooltip { position:fixed; display:none; background:#222; border:1px solid #aaa;
             padding:7px; white-space:pre; font-size:12px; pointer-events:none; z-index:5; }
  #camera { min-width:0; min-height:0; object-fit:contain; display:block;
            place-self:center; margin:auto; background:#000; }
  #keyState { min-width:155px; color:#aaa; font-family:monospace; text-align:center; }
  #stateCards { display:grid; grid-template-columns:1fr 1fr; gap:5px; padding:4px 5px; }
  .stateCard { min-width:0; padding:7px; text-align:center; font-weight:900;
               font-size:clamp(18px, 2.5vw, 34px); letter-spacing:.08em;
               border:2px solid #555; border-radius:6px; background:#222; }
  .stateCard.good { color:#75ff91; border-color:#28c76f; background:#10341d; }
  .stateCard.warn { color:#ffe06b; border-color:#d6a900; background:#3a3108; }
  .stateCard.bad { color:#ff8c85; border-color:#ff3b30; background:#461411; }
  .stateCard.info { color:#78c4ff; border-color:#2d8cff; background:#102a43; }
  @media (max-width:700px) {
    #content { grid-template-columns:minmax(0, 3fr) minmax(0, 2fr); gap:2px; padding:0 2px 2px; }
    #toolbar { padding:3px; gap:2px; }
    button, input { padding:4px 6px; }
    #navStatus { padding:4px 6px; font-size:11px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
    #driveArea { gap:2px; flex-direction:column; }
    #keyState { font-size:10px; min-width:0; }
  }
</style>
</head>
<body>
  <div id="navStatus">nav: —</div>
  <div id="stateCards"><div id="navCard" class="stateCard">NAV: —</div>
    <div id="mappingCard" class="stateCard">MAPPING: IDLE</div></div>
  <div id="toolbar">
    <button id="resetBtn">Reset all</button>
    <button id="clearMarkersBtn">Clear markers</button>
    <button id="moveBtn">Move</button>
    <button id="clearBtn">Clear goal</button>
    <button id="traceToggleBtn">Hide trace</button>
    <button id="mappingStartBtn">Start mapping</button>
    <button id="mappingFinishBtn">Finish mapping</button>
    <button id="mappingResumeBtn">Resume checkpoint</button>
    <input id="semanticClass" value="chair" size="10"><button id="semanticGoBtn">Go to nearest</button>
    <a href="/mapping_log" style="color:#8cf">Download mapping log</a>
    <span id="status"></span>
  </div>
  <div id="content">
    <section id="mapSection">
      <div id="hint">Map: click its right half to set a goal</div>
      <div id="mapWrap"><img id="stream" src="/stream"><canvas id="traceCanvas"></canvas></div>
    </section>
    <section id="cameraSection">
      <div id="hint">Camera: click an obstacle ground-contact point</div>
      <img id="camera" src="/camera_stream">
      <div id="driveArea">
        <div id="dpad">
          <div></div><button class="drive" data-dir="up">W</button><div></div>
          <button class="drive" data-dir="left">A</button><div></div>
          <button class="drive" data-dir="right">D</button>
          <div></div><button class="drive" data-dir="down">S</button><div></div>
        </div>
        <div id="keyState">WASD: stopped</div>
      </div>
    </section>
    <div id="tooltip"></div>
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
      flash((await post('/reset')) ? 'all state reset' : 'failed');
    };
    document.getElementById('clearMarkersBtn').onclick = async () => {
      flash((await post('/markers/clear')) ? 'markers cleared' : 'failed');
    };
    document.getElementById('moveBtn').onclick = async () => {
      flash((await post('/goal/move')) ? 'moving...' : 'failed');
    };
    document.getElementById('clearBtn').onclick = async () => {
      flash((await post('/goal/clear')) ? 'goal cleared' : 'failed');
    };
    let traceVisible = true;
    document.getElementById('traceToggleBtn').onclick = () => {
      traceVisible = !traceVisible;
      document.getElementById('traceCanvas').style.display = traceVisible ? 'block' : 'none';
      document.getElementById('traceToggleBtn').textContent = traceVisible ? 'Hide trace' : 'Show trace';
    };
    document.getElementById('mappingStartBtn').onclick = async () => {
      flash((await post('/mapping/start')) ? 'mapping started' : 'failed');
    };
    document.getElementById('mappingFinishBtn').onclick = async () => {
      flash((await post('/mapping/finish')) ? 'saving map...' : 'failed');
    };
    document.getElementById('mappingResumeBtn').onclick = async () => {
      flash((await post('/mapping/resume')) ? 'relocalizing checkpoint' : 'failed');
    };
    document.getElementById('semanticGoBtn').onclick = async () => {
      const label = document.getElementById('semanticClass').value.trim();
      flash((await post('/semantic/go', {class: label})) ? 'semantic goal sent' : 'failed');
    };

    // Right half of the combined stream is the map panel — click it to set a goal.
    async function setMapGoalFromClick(e) {
      const rect = document.getElementById('stream').getBoundingClientRect();
      const fx = (e.clientX - rect.left) / rect.width;
      const fy = (e.clientY - rect.top) / rect.height;
      if (fx < 0.5) {
        flash('click the map panel (right side) to set a goal');
        return;
      }
      const ok = await post('/goal/set', {fx: (fx - 0.5) * 2, fy: fy});
      flash(ok ? 'goal set' : 'failed');
    }
    document.getElementById('stream').addEventListener('click', setMapGoalFromClick);

    document.getElementById('camera').addEventListener('click', async (e) => {
      const rect = e.target.getBoundingClientRect();
      const u = (e.clientX - rect.left) / rect.width;
      const v = (e.clientY - rect.top) / rect.height;
      const ok = await post('/obstacle/mark', {u_fraction: u, v_fraction: v});
      flash(ok ? 'obstacle marked' : 'projection failed; check calibration');
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

    // Keyboard drive supports simultaneous keys. Re-send while held to stay
    // inside drive_node's command timeout, and always stop on blur/visibility loss.
    const driveKeys = new Set();
    const keyState = document.getElementById('keyState');
    function sendKeyboardDrive() {
      const forward = (driveKeys.has('w') ? 1 : 0) - (driveKeys.has('s') ? 1 : 0);
      const turn = (driveKeys.has('a') ? 1 : 0) - (driveKeys.has('d') ? 1 : 0);
      const active = [...driveKeys].map(k => k.toUpperCase()).sort().join('+');
      keyState.textContent = active ? 'WASD: ' + active : 'WASD: stopped';
      post('/drive/vector', {forward, turn});
    }
    function isTyping() {
      const tag = document.activeElement && document.activeElement.tagName;
      return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT';
    }
    document.addEventListener('keydown', (e) => {
      const key = e.key.toLowerCase();
      if (!'wasd'.includes(key) || isTyping()) return;
      e.preventDefault();
      if (!driveKeys.has(key)) { driveKeys.add(key); sendKeyboardDrive(); }
    });
    document.addEventListener('keyup', (e) => {
      const key = e.key.toLowerCase();
      if (!'wasd'.includes(key)) return;
      e.preventDefault(); driveKeys.delete(key); sendKeyboardDrive();
    });
    setInterval(() => { if (driveKeys.size) sendKeyboardDrive(); }, 180);
    function keyboardStop() {
      if (driveKeys.size) { driveKeys.clear(); sendKeyboardDrive(); }
      else post('/drive/vector', {forward:0, turn:0});
    }
    window.addEventListener('blur', keyboardStop);
    document.addEventListener('visibilitychange', () => { if (document.hidden) keyboardStop(); });

    // Poll the current navigation status + live obstacle-block state.
    const navStatus = document.getElementById('navStatus');
    const navCard = document.getElementById('navCard');
    const mappingCard = document.getElementById('mappingCard');
    function stateClass(value, kind) {
      const text = String(value).toUpperCase();
      if (text.includes('FAIL') || text.includes('INVALID') || text.includes('ABORT')) return 'bad';
      if (text.includes('RECOVER') || text.includes('RETURN') || text.includes('BLOCK') ||
          text.includes('LOCALIZ') || text.includes('SAVING')) return 'warn';
      if (text.includes('REACHED') || text === 'WORKING' || text === 'MAPPING') return 'good';
      return kind === 'mapping' && text === 'SAVING_MAP' ? 'info' : '';
    }
    async function pollNavStatus() {
      try {
        const resp = await fetch('/nav_status');
        const data = await resp.json();
        let text = 'nav: ' + data.status + '  |  mapping: ' + data.mapping_state + ' (' + data.mapping_detail + ')';
        if (data.obstacle_blocked) text += '  |  WARNING: obstacle blocking forward motion';
        navStatus.textContent = text;
        navStatus.classList.toggle('obstacle', !!data.obstacle_blocked);
        navCard.textContent = String(data.status).toUpperCase();
        mappingCard.textContent = String(data.mapping_state).toUpperCase();
        navCard.className = 'stateCard ' + stateClass(data.status, 'nav');
        mappingCard.className = 'stateCard ' + stateClass(data.mapping_state, 'mapping');
      } catch (e) { /* keep last known text on a transient fetch failure */ }
    }
    pollNavStatus();
    setInterval(pollNavStatus, 500);

    const canvas = document.getElementById('traceCanvas');
    const tooltip = document.getElementById('tooltip');
    let tracePoints = [];
    async function updateTrace() {
      try {
        const data = await (await fetch('/mapping_trace')).json();
        const rect = document.getElementById('stream').getBoundingClientRect();
        canvas.width = Math.max(1, Math.round(rect.width));
        canvas.height = Math.max(1, Math.round(rect.height));
        const ctx = canvas.getContext('2d');
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        tracePoints = traceVisible ? (data.points || []) : [];
        tracePoints.forEach((p, i) => {
          p.px = p.fx * canvas.width; p.py = p.fy * canvas.height;
          if (i && tracePoints[i-1].fx != null && p.fx != null) {
            ctx.strokeStyle = '#28c76f'; ctx.lineWidth = 2; ctx.beginPath();
            ctx.moveTo(tracePoints[i-1].px, tracePoints[i-1].py); ctx.lineTo(p.px, p.py); ctx.stroke();
          }
          ctx.fillStyle = p.color; ctx.beginPath(); ctx.arc(p.px, p.py, p.radius || 4, 0, 2*Math.PI); ctx.fill();
        });
      } catch (e) {}
    }
    canvas.onmousemove = (e) => {
      const rect = canvas.getBoundingClientRect();
      const x = e.clientX - rect.left, y = e.clientY - rect.top;
      const point = tracePoints
        .filter(p => Math.hypot(p.px-x, p.py-y) < 10)
        .sort((a,b) => (b.priority-a.priority) ||
              (Math.hypot(a.px-x,a.py-y)-Math.hypot(b.px-x,b.py-y)))[0];
      if (!point) { tooltip.style.display='none'; return; }
      tooltip.textContent = point.tooltip; tooltip.style.display='block';
      tooltip.style.left=(e.clientX+12)+'px'; tooltip.style.top=(e.clientY+12)+'px';
    };
    canvas.onmouseleave = () => tooltip.style.display='none';
    canvas.onclick = setMapGoalFromClick;
    function fitMedia() {
      const mapHost = document.getElementById('mapSection');
      const mapHint = mapHost.querySelector('#hint');
      const mw = mapHost.clientWidth, mh = Math.max(1, mapHost.clientHeight-mapHint.offsetHeight);
      const mapWidth = Math.min(mw, mh * (1280/480));
      const mapHeight = mapWidth / (1280/480);
      const mapWrap = document.getElementById('mapWrap');
      mapWrap.style.width = mapWidth+'px'; mapWrap.style.height = mapHeight+'px';
      const cameraHost = document.getElementById('cameraSection');
      const controls = document.getElementById('driveArea');
      const ch = Math.max(1, cameraHost.clientHeight-cameraHost.querySelector('#hint').offsetHeight-controls.offsetHeight);
      const cw = cameraHost.clientWidth;
      const cameraWidth = Math.min(cw, ch*(4/3));
      const cameraHeight = cameraWidth/(4/3);
      const camera = document.getElementById('camera');
      camera.style.width=cameraWidth+'px'; camera.style.height=cameraHeight+'px';
    }
    window.addEventListener('resize', fitMedia);
    new ResizeObserver(fitMedia).observe(document.getElementById('content'));
    fitMedia();
    updateTrace(); setInterval(updateTrace, 1000);
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
    obstacle_marks: list[tuple[float, float, str, float | None]],
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

    for x, y, label, _ in obstacle_marks:
        col = (x - grid.info.origin.position.x) / res
        row = (y - grid.info.origin.position.y) / res
        if 0 <= col < gw and 0 <= row < gh:
            marker = tuple(round(value) for value in to_display(x, y))
            cv2.drawMarker(img, marker, (0, 165, 255), cv2.MARKER_DIAMOND, 14, 2)
            cv2.putText(
                img, label, (marker[0] + 8, marker[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 100, 255), 1, cv2.LINE_AA,
            )

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
    latest_camera_jpeg: bytes | None = None
    lock = threading.Lock()
    node: "DashboardNode | None" = None

    # path -> (node, parsed_json_body) -> None. Kept as simple lambdas since
    # each just forwards to one DashboardNode method.
    _ROUTES = {
        "/reset": lambda node, body: node.reset_map(),
        "/markers/clear": lambda node, body: node.clear_markers(),
        "/goal/set": lambda node, body: node.set_goal_from_fraction(
            float(body.get("fx", 0.5)), float(body.get("fy", 0.5))
        ),
        "/goal/move": lambda node, body: node.move_to_goal(),
        "/goal/clear": lambda node, body: node.clear_goal(),
        "/mode/toggle": lambda node, body: node.toggle_mode(),
        "/drive": lambda node, body: node.drive(str(body.get("dir", "stop"))),
        "/drive/vector": lambda node, body: node.drive_vector(
            float(body.get("forward", 0.0)), float(body.get("turn", 0.0))
        ),
        "/obstacle/mark": lambda node, body: node.mark_obstacle(
            float(body.get("u_fraction", -1.0)), float(body.get("v_fraction", -1.0))
        ),
        "/mapping/start": lambda node, body: node.start_mapping(),
        "/mapping/finish": lambda node, body: node.mapping_command("FINISH"),
        "/mapping/resume": lambda node, body: node.mapping_command("RESUME"),
        "/semantic/go": lambda node, body: node.semantic_goal(str(body.get("class", "chair"))),
    }

    def log_message(self, *args) -> None:
        pass

    def do_GET(self) -> None:
        if self.path == "/stream":
            self._serve_stream()
        elif self.path == "/camera_stream":
            self._serve_stream(camera=True)
        elif self.path == "/nav_status":
            self._serve_nav_status()
        elif self.path == "/mapping_trace":
            self._serve_json(_MjpegHandler.node.mapping_trace() if _MjpegHandler.node else {"points": []})
        elif self.path == "/mapping_log":
            self._serve_mapping_log()
        else:
            self._serve_index()

    def _serve_nav_status(self) -> None:
        node = _MjpegHandler.node
        payload = {
            "status": node._nav_status if node else "Unknown",
            "obstacle_blocked": node._obstacle_blocked if node else False,
            "mapping_state": node._mapping_status.get("state", "UNKNOWN") if node else "UNKNOWN",
            "mapping_detail": node._mapping_status.get("detail", "") if node else "",
        }
        self._serve_json(payload)

    def _serve_json(self, payload: dict) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_index(self) -> None:
        body = _INDEX_HTML.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_mapping_log(self) -> None:
        node = _MjpegHandler.node
        session = node.latest_session_dir() if node else None
        path = session / "full_log.jsonl" if session else None
        if path is None or not path.exists():
            self.send_response(404); self.end_headers(); return
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "application/x-ndjson")
        self.send_header("Content-Disposition", f'attachment; filename="{session.name}_full_log.jsonl"')
        self.send_header("Content-Length", str(len(body)))
        self.end_headers(); self.wfile.write(body)

    def _serve_stream(self, camera: bool = False) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _MjpegHandler.lock:
                    data = (
                        _MjpegHandler.latest_camera_jpeg
                        if camera else _MjpegHandler.latest_jpeg
                    )
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
        except ValueError as exc:
            node.get_logger().warn(f"Rejected dashboard request {self.path}: {exc}")
            resp_body = str(exc).encode("utf-8")
            self.send_response(400)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(resp_body)))
            self.end_headers()
            self.wfile.write(resp_body)
            return
        except Exception:
            node.get_logger().error(f"dashboard POST {self.path} failed unexpectedly")
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
        self._nav_status = "Not started"
        self._obstacle_blocked = False
        self._mapping_status: dict = {"state": "IDLE", "detail": "monitor unavailable"}
        self._trace_suppressed = False
        self._trace_session_id: str | None = None
        self._camera_image: np.ndarray | None = None
        self._obstacle_marks: list[tuple[float, float, str, float | None]] = []
        self._detections: list[dict] = []

        self.declare_parameter("camera_fx", 0.0)
        self.declare_parameter("camera_fy", 0.0)
        self.declare_parameter("camera_cx", 0.0)
        self.declare_parameter("camera_cy", 0.0)
        self.declare_parameter("camera_k1", 0.0)
        self.declare_parameter("camera_k2", 0.0)
        self.declare_parameter("camera_p1", 0.0)
        self.declare_parameter("camera_p2", 0.0)
        self.declare_parameter("camera_height_m", 0.0)
        self.declare_parameter("camera_x_m", 0.0)
        self.declare_parameter("camera_y_m", 0.0)
        self.declare_parameter("camera_pitch_down_deg", 0.0)
        self.declare_parameter("camera_yaw_left_deg", 0.0)
        self.declare_parameter("automatic_marker_ttl_s", 3.0)
        self.declare_parameter("mapping_session_root", "/home/konkon/dev/autonomous-rover/mapping_sessions")
        self._automatic_marker_ttl = float(
            self.get_parameter("automatic_marker_ttl_s").value
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/rover/camera/image_raw", self._on_camera_image, qos_profile_sensor_data
        )
        self.create_subscription(
            OccupancyGrid, "/map", self._on_occupancy_grid, qos_profile_sensor_data
        )
        self.create_subscription(String, "/rover/current_mode", self._on_mode, _MODE_QOS)
        self.create_subscription(
            Range, "/rover/ultrasonic/range", self._on_range, qos_profile_sensor_data
        )
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(
            Bool, "/rover/obstacle_blocked", self._on_obstacle_blocked, _MODE_QOS
        )
        self.create_subscription(
            String, "/rover/object_detections", self._on_object_detections, 10
        )
        self.create_subscription(String, "/rover/semantic_objects", self._on_semantic_objects, 10)
        self.create_subscription(String, "/rover/mapping_status", self._on_mapping_status, 10)

        self._mode_pub = self.create_publisher(String, "/rover/mode", _CMD_QOS)
        self._mapping_control_pub = self.create_publisher(String, "/rover/mapping_control", 10)
        self._semantic_goal_pub = self.create_publisher(String, "/rover/semantic_goal", 10)
        self._teleop_pub = self.create_publisher(Twist, "/rover/cmd_vel_teleop", _CMD_QOS)
        self._compute_path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self._navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_timer(0.1, self._render)
        self.create_timer(2.0, self._log_pose)
        self.get_logger().info(f"Dashboard ready — open http://raspberrypi.local:{PORT}")

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_camera_image(self, msg: Image) -> None:
        if msg.encoding not in ("rgb8", "bgr8") or msg.step < msg.width * 3:
            self.get_logger().warn(
                f"Unsupported camera encoding/step: {msg.encoding}, step={msg.step}",
                throttle_duration_sec=5.0,
            )
            return
        rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
        frame = rows[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        self._camera_image = (
            cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else frame.copy()
        )

    def _on_occupancy_grid(self, msg: OccupancyGrid) -> None:
        self._occupancy_grid = msg

    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data

    def _on_plan(self, msg: Path) -> None:
        self._nav_path = msg

    def _on_range(self, msg: Range) -> None:
        self._ultrasonic_range = float(msg.range)
        self._ultrasonic_stamp = time.monotonic()

    def _on_obstacle_blocked(self, msg: Bool) -> None:
        self._obstacle_blocked = msg.data

    def _on_mapping_status(self, msg: String) -> None:
        try:
            self._mapping_status = json.loads(msg.data)
            session_id = self._mapping_status.get("session_id")
            if session_id and session_id != self._trace_session_id:
                self._trace_session_id = session_id
                self._trace_suppressed = False
        except json.JSONDecodeError:
            self._mapping_status = {"state": "UNKNOWN", "detail": msg.data}

    def mapping_command(self, command: str) -> None:
        if command == "FINISH" and self._mapping_status.get("state") != "MAPPING":
            raise ValueError("mapping is not currently running")
        self._mapping_control_pub.publish(String(data=command))

    def start_mapping(self) -> None:
        self._clear_runtime_display()
        self._mapping_control_pub.publish(String(data="START"))

    def _clear_runtime_display(self) -> None:
        self._occupancy_grid = None
        self._obstacle_marks.clear()
        self._detections = []
        self._trace_suppressed = True
        self.clear_goal()
        self._publish_mode("MANUAL")

    def semantic_goal(self, label: str) -> None:
        if self._mapping_status.get("state") != "WORKING":
            raise ValueError("navigation is available only after localization is stable")
        if not label.strip():
            raise ValueError("semantic class cannot be empty")
        self._publish_mode("AUTO")
        self._semantic_goal_pub.publish(String(data=label.strip()))

    def mapping_trace(self) -> dict:
        grid = self._occupancy_grid
        if self._trace_suppressed:
            return {"status": self._mapping_status, "points": []}
        session = self.latest_session_dir()
        session_dir = str(session) if session else None
        if grid is None or not session_dir:
            return {"status": self._mapping_status, "points": []}
        try:
            summary = json.loads((FilePath(session_dir) / "summary.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {"status": self._mapping_status, "points": []}
        points = []
        for item in summary.get("points", []):
            if item.get("x") is None or item.get("y") is None:
                continue
            col = (item["x"] - grid.info.origin.position.x) / grid.info.resolution
            row = (item["y"] - grid.info.origin.position.y) / grid.info.resolution
            fx = 0.5 + 0.5 * col / max(1, grid.info.width)
            fy = (grid.info.height - 1 - row) / max(1, grid.info.height)
            event, status = item.get("event", ""), item.get("status", "")
            color = "#28c76f"
            if event == "CHECKPOINT": color = "#2d8cff"
            elif status == "abnormal" or "TIMEOUT" in event: color = "#ff3b30"
            elif event not in ("HEALTH_SAMPLE", "MAPPING_STARTED"): color = "#ffcc00"
            reasons = ", ".join(r.get("code", "") for r in item.get("reasons", [])) or "none"
            tooltip = (f"time: {item.get('time')}\nstate: {item.get('state')}\n"
                       f"event: {event}\nreasons: {reasons}\naction: {item.get('action', '')}")
            points.append({"fx": fx, "fy": fy, "color": color,
                           "radius": 6 if event != "HEALTH_SAMPLE" else 3,
                           "priority": 2 if event == "CHECKPOINT" else
                                       (1 if event != "HEALTH_SAMPLE" else 0),
                           "tooltip": tooltip})
        return {"status": self._mapping_status, "points": points}

    def latest_session_dir(self) -> FilePath | None:
        active = self._mapping_status.get("session_dir")
        if active:
            return FilePath(active)
        root = FilePath(str(self.get_parameter("mapping_session_root").value)).expanduser()
        sessions = sorted(root.glob("session_*/summary.json"), reverse=True)
        return sessions[0].parent if sessions else None

    def _on_object_detections(self, msg: String) -> None:
        try:
            detections = json.loads(msg.data)
            if not isinstance(detections, list):
                raise ValueError("detection payload is not a list")
            self._detections = detections
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Rejected object detections: {exc}")

    def _on_semantic_objects(self, msg: String) -> None:
        try:
            objects = json.loads(msg.data)
            manual = [marker for marker in self._obstacle_marks if marker[3] is None and
                      not marker[2].startswith("semantic:")]
            semantic = [
                (float(obj["x"]), float(obj["y"]),
                 f"semantic:{obj['class']}_{obj['id']} {obj['status']}", float("inf"))
                for obj in objects if obj.get("status") == "confirmed"
            ]
            self._obstacle_marks = manual + semantic
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Rejected semantic objects: {exc}")

    def reset_map(self) -> None:
        """Restore the complete rover mapping/navigation UI to startup state."""
        self.get_logger().warn("Reset all requested — returning to startup state")
        self._clear_runtime_display()
        self._mapping_control_pub.publish(String(data="RESET"))

    def clear_markers(self) -> None:
        """Clear camera-derived map markers without resetting SLAM."""
        self._obstacle_marks.clear()
        self.get_logger().info("All camera obstacle markers cleared")

    def mark_obstacle(
        self,
        u_fraction: float,
        v_fraction: float,
        label: str = "obstacle",
        merge_label: str | None = None,
    ) -> None:
        """Project a camera click to the ground and retain it in the current map."""
        image = self._camera_image
        pose = self._lookup_pose()
        if image is None or pose is None or not (0.0 <= u_fraction <= 1.0) or not (
            0.0 <= v_fraction <= 1.0
        ):
            raise ValueError("camera image, map pose, or click position unavailable")

        values = {
            name: float(self.get_parameter(name).value)
            for name in (
                "camera_fx", "camera_fy", "camera_cx", "camera_cy",
                "camera_k1", "camera_k2", "camera_p1", "camera_p2", "camera_height_m",
                "camera_x_m", "camera_y_m", "camera_pitch_down_deg", "camera_yaw_left_deg",
            )
        }
        raw_pixel = np.array(
            [[[u_fraction * (image.shape[1] - 1), v_fraction * (image.shape[0] - 1)]]],
            dtype=np.float64,
        )
        camera_matrix = np.array(
            [
                [values["camera_fx"], 0.0, values["camera_cx"]],
                [0.0, values["camera_fy"], values["camera_cy"]],
                [0.0, 0.0, 1.0],
            ]
        )
        distortion = np.array(
            [values["camera_k1"], values["camera_k2"], values["camera_p1"], values["camera_p2"]]
        )
        undistorted = cv2.undistortPoints(
            raw_pixel, camera_matrix, distortion, P=camera_matrix
        )[0, 0]
        point = pixel_to_ground(
            float(undistorted[0]),
            float(undistorted[1]),
            fx=values["camera_fx"], fy=values["camera_fy"],
            cx=values["camera_cx"], cy=values["camera_cy"],
            camera_height_m=values["camera_height_m"],
            camera_x_m=values["camera_x_m"], camera_y_m=values["camera_y_m"],
            camera_pitch_down_deg=values["camera_pitch_down_deg"],
            camera_yaw_left_deg=values["camera_yaw_left_deg"],
        )
        if point is None:
            raise ValueError("clicked ray does not intersect the ground in front of camera")

        yaw = math.radians(pose[2])
        map_x = pose[0] + math.cos(yaw) * point[0] - math.sin(yaw) * point[1]
        map_y = pose[1] + math.sin(yaw) * point[0] + math.cos(yaw) * point[1]
        if merge_label is not None:
            merge_index = next(
                (
                    index
                    for index, (old_x, old_y, old_label, _) in enumerate(
                        self._obstacle_marks
                    )
                    if old_label.startswith(merge_label + " ")
                    and math.hypot(map_x - old_x, map_y - old_y) < 0.25
                ),
                None,
            )
            if merge_index is not None:
                old_x, old_y, _, _ = self._obstacle_marks[merge_index]
                self._obstacle_marks[merge_index] = (
                    0.7 * old_x + 0.3 * map_x,
                    0.7 * old_y + 0.3 * map_y,
                    label,
                    time.monotonic(),
                )
            else:
                self._obstacle_marks.append((map_x, map_y, label, time.monotonic()))
        else:
            self._obstacle_marks.append((map_x, map_y, label, None))
        self.get_logger().info(
            f"Obstacle: base=({point[0]:.2f}, {point[1]:.2f})m, "
            f"map=({map_x:.2f}, {map_y:.2f})m"
        )

    def set_goal_from_fraction(self, fx: float, fy: float) -> None:
        """Convert a click on the map panel (fractional x/y, 0..1) to a map-frame goal."""
        if self._mapping_status.get("state") != "WORKING":
            raise ValueError("set navigation goals only after localization is stable")
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
            self._nav_status = "Path planning failed"
            return
        self._nav_status = "Path ready; waiting to move"
        # No further action needed — planner_server publishes the resulting
        # path to /plan itself, which _on_plan/_render already pick up.
        goal_handle.get_result_async()

    def move_to_goal(self) -> None:
        """Send the last-set goal as a real NavigateToPose action, forcing AUTO mode."""
        if self._mapping_status.get("state") != "WORKING":
            raise ValueError("navigation is available only after localization is stable")
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
            self._nav_status = "Navigation rejected"
            return
        self._nav_goal_handle = goal_handle
        self._nav_status = "Navigating"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigate_result)

    def _on_navigate_result(self, future) -> None:
        self._nav_goal_handle = None
        status = future.result().status
        self.get_logger().info(f"navigate_to_pose finished: status={status}")
        self._nav_status = _GOAL_STATUS_TEXT.get(status, f"Unknown status ({status})")
        # Keep the map: mapping_monitor_node now detects corruption and owns
        # checkpoint recovery. Resetting after every goal destroys route and
        # semantic-map continuity.

    def clear_goal(self) -> None:
        """Cancel any in-flight navigation and clear the local goal/path state."""
        self._goal_pose = None
        self._nav_path = None
        self._nav_status = "Not started"
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

    def drive_vector(self, forward: float, turn: float) -> None:
        """Drive from normalized WASD axes, allowing diagonal key combinations."""
        forward = max(-1.0, min(1.0, forward))
        turn = max(-1.0, min(1.0, turn))
        self._publish_mode("MANUAL")
        msg = Twist()
        msg.linear.x = 0.2 * forward
        msg.angular.z = turn
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
        marker_cutoff = time.monotonic() - self._automatic_marker_ttl
        self._obstacle_marks = [
            marker
            for marker in self._obstacle_marks
            if marker[3] is None or marker[3] >= marker_cutoff
        ]
        pose = self._lookup_pose()

        scan_panel = _render_scan_panel(self._scan, PANEL_H, PANEL_W)
        occupancy_panel = _render_occupancy_panel(
            self._occupancy_grid, pose, self._nav_path, self._obstacle_marks, PANEL_H, PANEL_W
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
            if self._camera_image is not None:
                camera = self._camera_image.copy()
                height, width = camera.shape[:2]
                for detection in self._detections:
                    x1 = round(float(detection["x1"]) * width)
                    y1 = round(float(detection["y1"]) * height)
                    x2 = round(float(detection["x2"]) * width)
                    y2 = round(float(detection["y2"]) * height)
                    label = f'{detection["label"]} {float(detection["confidence"]):.0%}'
                    cv2.rectangle(camera, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(camera, ((x1 + x2) // 2, y2), 5, (0, 165, 255), -1)
                    cv2.putText(
                        camera, label, (x1, max(16, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
                    )
                cv2.putText(
                    camera, "click obstacle ground-contact point", (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA,
                )
                _, camera_jpeg = cv2.imencode(
                    ".jpg", camera, [cv2.IMWRITE_JPEG_QUALITY, 80]
                )
                _MjpegHandler.latest_camera_jpeg = camera_jpeg.tobytes()


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
