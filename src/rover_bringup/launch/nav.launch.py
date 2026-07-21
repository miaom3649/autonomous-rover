import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim = LaunchConfiguration("use_sim")

    home = os.path.expanduser("~")
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
                default_value="http://192.168.1.151:8765/depth",
                description="URL of the Windows depth inference server",
            ),
            DeclareLaunchArgument(
                "dashboard_start_delay",
                default_value="3.0",
                description="Seconds to wait before starting dashboard_node.",
            ),
            DeclareLaunchArgument(
                "nav2_start_delay",
                default_value="15.0",
                description=(
                    "Seconds to wait before starting the Nav2 stack, so the first "
                    "stop-cycle VO fix has already landed before Nav2's costmaps "
                    "start looking for map->odom->base_link."
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
                parameters=[{"use_sim": use_sim, "frame_width": 320, "frame_height": 240}],
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
            # ── Visual odometry (frame-to-frame PnP, triggered per stop cycle) ─────
            Node(
                package="rover_navigation",
                executable="vo_node",
                name="vo_node",
                output="screen",
            ),
            # ── TF: map -> odom, static identity ────────────────────────────────────
            # No absolute correction source (no lidar, no mapping) — vo_node's
            # accumulated pose is the whole story, so map and odom coincide.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            ),
            # ── Stop-and-go filter (bursts Nav2's cmd_vel, waits for a fresh VO fix) ─
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
