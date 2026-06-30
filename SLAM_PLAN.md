# ORB-SLAM3 Integration Plan

Goal: enable the rover to build a map of an unknown environment, localize itself within it, and navigate autonomously to a given coordinate.

---

## Steps

### Step 1: Camera Driver Node
- Location: `src/rover_base/rover_base/camera_node.py`
- Publishes `/rover/camera/image_raw` and `/rover/camera/camera_info`
- Supports `use_sim:=true` mode (publishes static test frames)
- Status: **Done**

### Step 2: Camera Calibration
- Captured frames at 640x480 via `capture_calibration_frames.py`, scaled intrinsics ×0.5 for 320x240 output
- Output: `config/camera.yaml` (focal length, principal point, distortion coefficients)
- RMS reprojection error: 0.7344 px
- Status: **Done**

### Step 3: Install ORB-SLAM3 on the Pi
- `libORB_SLAM3.so` compiled successfully at `~/ORB_SLAM3/lib/`
- Lightweight ROS2 wrapper: `src/rover_slam/` (single .cpp, links libORB_SLAM3.so)
- Publishes `/orb_slam3/pose` (geometry_msgs/PoseStamped)
- Build: `colcon build --packages-select rover_slam --parallel-workers 1 --cmake-args -DORB_SLAM3_ROOT_DIR=$HOME/ORB_SLAM3 -DPangolin_DIR=$HOME/Pangolin/build`
- Status: **Code written — needs colcon build on Pi**

### Step 4: SLAM Pose Bridge + Config
- Location: `src/rover_navigation/rover_navigation/slam_pose_bridge.py`
- Subscribes to `/orb_slam3/pose`, republishes as `nav_msgs/Odometry` on `/rover/odom`
- Config: `config/orbslam3.yaml` (500 features, Pi 4 tuned)
- Status: **Done**

### Step 5: Nav2 Autonomous Navigation
- Install on Pi: `sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup`
- Config: `config/nav2_params.yaml` (PiCar-X footprint 21×16.5 cm, RPP controller, NavFn planner, Pi 4 tuned frequencies)
- Localization: ORB-SLAM3 provides map→odom→base_link TF (no lidar/AMCL needed)
- Status: **Done**

### Step 6: Launch Files
- Location: `src/rover_bringup/` (new package)
- `slam.launch.py` — mapping mode: camera + ultrasonic + ORB-SLAM3 + pose bridge + static map→odom TF
- `nav.launch.py` — navigation mode: all SLAM nodes + Nav2 stack (map_server, controller, planner, bt_navigator, lifecycle_manager)
- `slam_pose_bridge` updated to broadcast `odom→base_link` TF (required by Nav2 costmap)
- Status: **Done**

---

## Commands Available After Completion

```bash
# Mapping — drive the rover around manually to build a map
ros2 launch rover_bringup slam.launch.py

# Save the map
ros2 run nav2_map_server map_saver_cli -f ~/maps/room

# Autonomous navigation — rover drives itself to the target coordinate
ros2 launch rover_bringup nav.launch.py map:=~/maps/room.yaml
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{ pose: { pose: { position: { x: 1.5, y: 0.8 } } } }"
```

---

## Dependencies

Step 1 → Step 2 → Step 3 → Step 4 → Step 5 → Step 6 (sequential)

Steps 2 and 3 require physical hardware. All other steps can be written on the dev machine ahead of time.

---

## Known Issues

### Monocular Scale Ambiguity + Bundle Adjustment Drift

**Problem:** ORB-SLAM3 in monocular mode has no concept of absolute metric scale. All reported distances are unitless relative values. After the rover stops moving, ORB-SLAM3 runs Bundle Adjustment (BA) — a batch re-optimization of all historical keyframe poses and 3D map points — which retroactively rewrites previously reported positions to be more geometrically consistent. This causes the SLAM-reported position to "rise then shrink back" after movement, making it unreliable for metric navigation.

**Confirmed by experiment:** Same behavior observed whether the rover is moved manually or via `nav_forward.py` command — the root cause is BA, not the motion source.

**Current workaround:** Static `position_scale=9.6` parameter in `slam_pose_bridge.py`, calibrated once by hand. Doesn't fix BA drift, only corrects the one-time scale factor at startup.

---

## Improvement: Switch ORB-SLAM3 to RGBD Mode (Implemented)

Replace monocular SLAM with **RGBD SLAM** by supplying metric depth maps from
Depth-Anything-V2-Metric-Indoor-Small running on the Windows host GPU.
This eliminates monocular scale ambiguity at the source — every keyframe has
real-world depth constraints, so BA corrections are metrically accurate.

### Architecture

```
Pi (camera_node)
  → /rover/camera/image_raw  ─────────────────────────────┐
                                                           │
Pi (depth_bridge_node)                                     │
  ← /rover/camera/image_raw (local ROS2 topic, same machine)│
  → POST JPEG over LAN → Windows host :8765/depth          │
  ← float32 depth map (meters, ~100 ms round trip)        │
  → /rover/camera/depth (32FC1, same timestamp as source)  │
                                                           ▼
Pi (orb_slam3_node RGBD)
  ← /rover/camera/image_raw + /rover/camera/depth (ApproximateTime sync)
  → TrackRGBD(rgb, depth, ts) → /orb_slam3/pose
```

`depth_bridge_node` runs on the Pi (not the dev machine) — the dev machine VM has no
ROS2 installed, and the Pi already has ROS2, so it's simpler for the Pi to talk to
the Windows depth server directly over plain HTTP. No ROS2 networking between VM
and Pi is needed for this feature.

### Why the stop-and-go architecture makes this simple

The rover moves 0.5 s, stops 0.9 s, camera settle_delay=0.5 s.
Depth round trip ≈ 100 ms — fits inside the 0.9 s stop window with 800 ms to spare.
No async pipeline, no extra stopping, one depth request per stop cycle (~0.7 Hz effective).

### Files changed

| File | Change |
|------|--------|
| `scripts/windows_depth_server/depth_server.py` | FastAPI server (NEW) |
| `scripts/windows_depth_server/requirements.txt` | Windows venv deps (NEW) |
| `src/rover_navigation/rover_navigation/depth_bridge_node.py` | Pi bridge node (NEW) |
| `src/rover_slam/src/orb_slam3_node.cpp` | MONOCULAR → RGBD + message_filters sync |
| `src/rover_slam/CMakeLists.txt` | Added message_filters |
| `src/rover_slam/package.xml` | Added message_filters |
| `config/orbslam3.yaml` | Added Camera.bf, ThDepth, DepthMapFactor |
| `src/rover_navigation/setup.py` | Registered depth_bridge_node entry point |

### Startup sequence

```bash
# 1. Windows host — activate venv and start depth server
depth_venv\Scripts\activate
python scripts\windows_depth_server\depth_server.py

# 2. Pi — start SLAM and navigation (depth_bridge_node is launched automatically)
ros2 launch rover_bringup nav.launch.py depth_server_url:=http://192.168.1.151:8765/depth
```

No ROS2 networking between the VM and the Pi is required for this feature —
`depth_bridge_node` runs on the Pi and reaches the Windows depth server over
plain HTTP on the LAN.

### orbslam3.yaml depth parameters

- `Camera.bf: 24.0` — virtual baseline × fx (≈7.5 cm equivalent)
- `ThDepth: 40.0` — mThDepth = 24×40/321 ≈ 3.0 m close-point threshold
- `DepthMapFactor: 1.0` — depth images are already float32 in meters, no conversion
