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
- Install: `ros-humble-navigation2`
- Config: `config/nav2_params.yaml` (robot footprint, costmap, planner parameters)
- Status: **TODO**

### Step 6: Launch Files
- Location: `src/rover_bringup/` (new package)
- `slam.launch.py` — mapping mode: camera + ultrasonic + ORB-SLAM3 + pose bridge
- `nav.launch.py` — navigation mode: load existing map + Nav2
- Status: **TODO**

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
