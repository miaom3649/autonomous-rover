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
- Use `ros2 run camera_calibration cameracalibrator` with a checkerboard target
- Output: `config/camera.yaml` (focal length, principal point, distortion coefficients)
- Status: **TODO — requires physical hardware**

### Step 3: Install ORB-SLAM3 on the Pi
- Build ORB-SLAM3 from source + `ORB-SLAM3-ROS2` wrapper
- Dependencies: OpenCV, Eigen3, Pangolin
- Status: **TODO — run on Pi, ~1 hour build time**

### Step 4: SLAM Pose Bridge + Config
- Location: `src/rover_navigation/rover_navigation/slam_pose_bridge.py`
- Subscribes to raw ORB-SLAM3 pose output, republishes as `nav_msgs/Odometry` on `/rover/odom`
- Config: `config/orbslam3.yaml` (feature count and other Pi 4 tuning parameters)
- Status: **TODO**

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
