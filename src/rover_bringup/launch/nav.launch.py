import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim = LaunchConfiguration("use_sim")

    home = os.path.expanduser("~")
    vocab_path = os.path.join(home, "ORB_SLAM3", "Vocabulary", "ORBvoc.txt")
    settings_path = os.path.join(home, "dev", "autonomous-rover", "config", "orbslam3.yaml")
    base_params = os.path.join(home, "dev", "autonomous-rover", "config", "base_params.yaml")
    nav2_params = os.path.join(home, "dev", "autonomous-rover", "config", "nav2_params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim",
                default_value="false",
                description="Run in simulation mode (no hardware required)",
            ),
            DeclareLaunchArgument(
                "depth_server_url",
                default_value="http://192.168.3.33:8765/depth",
                description="URL of the Windows depth inference server",
            ),
            DeclareLaunchArgument(
                "dashboard_start_delay",
                default_value="5.0",
                description=(
                    "Seconds to wait before starting dashboard_node. It's lightweight, "
                    "but starting it at t=0 alongside ORB-SLAM3's vocabulary load would "
                    "still add to the same startup memory spike this file already "
                    "staggers around, so it gets a small delay of its own rather than none."
                ),
            ),
            DeclareLaunchArgument(
                "nav2_start_delay",
                default_value="45.0",
                description=(
                    "Seconds to wait before starting the Nav2 stack. On a 2GB Pi 4, "
                    "ORB-SLAM3 loading its ~145MB text vocabulary at the same time "
                    "Nav2's costmaps and the camera/SLAM/depth pipeline are all "
                    "initializing can exceed available RAM, causing the whole system "
                    "to swap-thrash into total unresponsiveness (SD-card-speed swap "
                    "under sustained pressure never triggers the OOM killer, so it "
                    "doesn't recover on its own). Staggering the startup keeps peak "
                    "memory demand from stacking."
                ),
            ),
            DeclareLaunchArgument(
                "drive_burst_s",
                default_value="2.0",
                description="Seconds Nav2's cmd_vel is passed through per drive burst.",
            ),
            # ── Hardware nodes ────────────────────────────────────────────────────
            Node(
                package="rover_base",
                executable="drive_node",
                name="drive_node",
                parameters=[base_params, {"use_sim": use_sim}],
            ),
            Node(
                package="rover_camera",
                executable="camera_node",
                name="camera_node",
                parameters=[
                    {
                        "use_sim": use_sim,
                        "frame_width": 320,
                        "frame_height": 240,
                        # Only ever one frame is used per stop cycle (see
                        # depth_bridge_node.py / orb_slam3_node) — a
                        # low rate cuts wasted capture/publish CPU and shrinks the
                        # window where a new frame could land mid-depth-round-trip
                        # and get mismatched against the wrong depth map.
                        "publish_rate_hz": 2.0,
                    }
                ],
            ),
            # ── Depth bridge (camera+AI depth, ultrasonic-corrected) ────────────────
            Node(
                package="rover_navigation",
                executable="depth_bridge_node",
                name="depth_bridge_node",
                parameters=[
                    {
                        "depth_server_url": LaunchConfiguration("depth_server_url"),
                        "settle_delay": 0.5,
                        "request_timeout": 0.8,
                        "ultrasonic_correction": True,
                    }
                ],
                output="screen",
            ),
            # ── ORB-SLAM3 (RGBD, primary pose source) ───────────────────────────────
            Node(
                package="rover_slam",
                executable="orb_slam3_node",
                name="orb_slam3_node",
                parameters=[
                    {
                        "vocab_path": vocab_path,
                        "settings_path": settings_path,
                    }
                ],
                output="screen",
                sigterm_timeout="3",
                sigkill_timeout="3",
            ),
            # ── SLAM pose bridge: publishes /rover/odom + odom->base_link TF from
            # ORB-SLAM3's pose directly. position_scale starts at 1.0 (not the
            # old, never-fully-diagnosed 9.6) pending fresh hand-push
            # recalibration now that depth is ultrasonic-corrected from the start.
            Node(
                package="rover_navigation",
                executable="slam_pose_bridge",
                name="slam_pose_bridge",
                parameters=[{"camera_tilt_deg": 2.0, "position_scale": 1.0}],
            ),
            # ── Ultrasonic → LaserScan (Nav2 costmap obstacle_layer observation source) ──
            Node(
                package="rover_navigation",
                executable="ultrasonic_to_scan_node",
                name="ultrasonic_to_scan_node",
            ),
            # ── TF: map -> odom, static identity ────────────────────────────────────
            # No absolute correction source (no lidar, no saved map) — the
            # cross-checked SLAM3 pose is the whole story, so map and odom
            # coincide.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            ),
            # ── Stop-and-go filter (bursts Nav2's cmd_vel, waits for a fresh fix) ────
            Node(
                package="rover_navigation",
                executable="stop_and_go_filter_node",
                name="stop_and_go_filter_node",
                parameters=[{"drive_burst_s": LaunchConfiguration("drive_burst_s")}],
            ),
            # ── Mode controller (MANUAL/AUTO arbitration + estop) ────────────────
            Node(
                package="rover_control",
                executable="mode_controller_node",
                name="mode_controller_node",
            ),
            # ── Live debug dashboard (camera/depth/mode/ultrasonic/position on :8082) ──
            TimerAction(
                period=LaunchConfiguration("dashboard_start_delay"),
                actions=[
                    Node(
                        package="rover_navigation",
                        executable="dashboard_node",
                        name="dashboard_node",
                    ),
                ],
            ),
            # ── Nav2 stack ────────────────────────────────────────────────────────
            # Delayed: see nav2_start_delay above. Output remapped to
            # /rover/cmd_vel_nav_raw — stop_and_go_filter_node bursts/gates it
            # into /rover/cmd_vel_nav, which mode_controller_node consumes.
            TimerAction(
                period=LaunchConfiguration("nav2_start_delay"),
                actions=[
                    Node(
                        package="nav2_controller",
                        executable="controller_server",
                        name="controller_server",
                        parameters=[nav2_params],
                        remappings=[("/cmd_vel", "/rover/cmd_vel_nav_raw")],
                    ),
                    Node(
                        package="nav2_planner",
                        executable="planner_server",
                        name="planner_server",
                        parameters=[nav2_params],
                    ),
                    Node(
                        package="nav2_behaviors",
                        executable="behavior_server",
                        name="behavior_server",
                        parameters=[nav2_params],
                        remappings=[("/cmd_vel", "/rover/cmd_vel_nav_raw")],
                    ),
                    Node(
                        package="nav2_bt_navigator",
                        executable="bt_navigator",
                        name="bt_navigator",
                        parameters=[nav2_params],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager",
                        parameters=[nav2_params],
                    ),
                ],
            ),
        ]
    )
