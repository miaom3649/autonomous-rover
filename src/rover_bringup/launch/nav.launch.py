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

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false",
                              description="Run in simulation mode (no hardware required)"),
        DeclareLaunchArgument("depth_server_url", default_value="http://192.168.1.151:8765/depth",
                              description="URL of the Windows depth inference server"),
        DeclareLaunchArgument(
            "nav2_start_delay", default_value="45.0",
            description=(
                "Seconds to wait before starting the Nav2 stack. On a 2GB Pi 4, "
                "ORB-SLAM3 loading its ~145MB text vocabulary at the same time Nav2's "
                "costmaps and camera/SLAM are all initializing can exceed available "
                "RAM, causing the whole system to swap-thrash into total unresponsiveness "
                "(SD-card-speed swap under sustained pressure never triggers the OOM "
                "killer, so it doesn't recover on its own). Staggering the startup keeps "
                "peak memory demand from stacking."
            ),
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
            parameters=[{"use_sim": use_sim, "frame_width": 320, "frame_height": 240}],
        ),
        # ── Depth bridge (grabs frames at stops, fetches metric depth from Windows GPU) ──
        Node(
            package="rover_navigation",
            executable="depth_bridge_node",
            name="depth_bridge_node",
            parameters=[{
                "depth_server_url": LaunchConfiguration("depth_server_url"),
                "settle_delay": 0.5,
                "request_timeout": 0.8,
            }],
            output="screen",
        ),

        # ── ORB-SLAM3 (visual odometry) ───────────────────────────────────────
        Node(
            package="rover_slam",
            executable="orb_slam3_node",
            name="orb_slam3_node",
            parameters=[{
                "vocab_path": vocab_path,
                "settings_path": settings_path,
            }],
            output="screen",
            sigterm_timeout="3",
            sigkill_timeout="3",
        ),
        Node(
            package="rover_navigation",
            executable="slam_pose_bridge",
            name="slam_pose_bridge",
            parameters=[{"camera_tilt_deg": 2.0, "position_scale": 9.6}],
        ),

        # ── SLAM initialisation helper (pan sweep until tracking starts) ─────
        Node(
            package="rover_navigation",
            executable="slam_init_helper_node",
            name="slam_init_helper_node",
        ),

        # ── Ultrasonic → LaserScan ────────────────────────────────────────────
        Node(
            package="rover_navigation",
            executable="ultrasonic_to_scan_node",
            name="ultrasonic_to_scan_node",
        ),

        # ── map→odom: static identity — ORB-SLAM3 is the only position source ──
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_static",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ),

        # ── Stop-and-go filter (gates Nav2 cmd_vel into move/pause bursts) ──
        Node(
            package="rover_navigation",
            executable="stop_and_go_filter_node",
            name="stop_and_go_filter_node",
            parameters=[{
                "move_duration": 0.5,
                "pause_duration": 0.9,
                "slam_loss_grace": 2.0,
                "backup_speed": 0.10,
                "backup_duration": 1.0,
                "slam_stabilize_duration": 5.0,
            }],
        ),

        # ── Mode controller (MANUAL/AUTO arbitration + estop) ────────────────
        Node(
            package="rover_control",
            executable="mode_controller_node",
            name="mode_controller_node",
            remappings=[("/rover/cmd_vel_nav", "/rover/cmd_vel_nav_gated")],
        ),

        # ── Nav2 stack ────────────────────────────────────────────────────────
        # Delayed: see nav2_start_delay above — avoids stacking Nav2's costmap
        # init memory spike on top of ORB-SLAM3's vocabulary load.
        TimerAction(
            period=LaunchConfiguration("nav2_start_delay"),
            actions=[
                Node(
                    package="nav2_controller",
                    executable="controller_server",
                    name="controller_server",
                    parameters=[nav2_params],
                    remappings=[("/cmd_vel", "/rover/cmd_vel_nav")],
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
                    remappings=[("/cmd_vel", "/rover/cmd_vel_nav")],
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
    ])
