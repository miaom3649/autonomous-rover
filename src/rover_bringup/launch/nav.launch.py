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
    lidar_params = os.path.join(
        home, "dev", "autonomous-rover", "src", "ydlidar_ros2_driver", "params", "X3.yaml"
    )
    slam_toolbox_params = os.path.join(
        home, "dev", "autonomous-rover", "config", "slam_toolbox_params.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false",
                              description="Run in simulation mode (no hardware required)"),
        DeclareLaunchArgument("depth_server_url", default_value="http://192.168.1.151:8765/depth",
                              description="URL of the Windows depth inference server"),
        DeclareLaunchArgument(
            "dashboard_start_delay", default_value="3.0",
            description="Seconds to wait before starting dashboard_node.",
        ),
        DeclareLaunchArgument(
            "nav2_start_delay", default_value="15.0",
            description=(
                "Seconds to wait before starting the Nav2 stack, so the lidar driver "
                "and slam_toolbox are already publishing map->odom before Nav2's "
                "costmaps start looking for it. Much shorter than the old ORB-SLAM3 "
                "era's delay (45s) — slam_toolbox has no ~145MB vocabulary file to "
                "load and is far lighter on startup memory."
            ),
        ),
        # Lidar mounting offset relative to base_link — PLACEHOLDER VALUES.
        # Measure the real offset once the lidar is permanently mounted and
        # update these defaults (or pass them as launch arguments).
        DeclareLaunchArgument("lidar_x", default_value="0.0", description="Lidar x offset (m)"),
        DeclareLaunchArgument("lidar_y", default_value="0.0", description="Lidar y offset (m)"),
        DeclareLaunchArgument("lidar_z", default_value="0.05", description="Lidar height (m)"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0",
                              description="Lidar yaw offset (rad) from base_link forward"),

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

        # ── 2D lidar (YDLidar X3) — primary localization sensor ────────────────
        Node(
            package="ydlidar_ros2_driver",
            executable="ydlidar_ros2_driver_node",
            name="ydlidar_ros2_driver_node",
            parameters=[lidar_params],
            output="screen",
        ),

        # ── Depth bridge (camera+AI depth for obstacle awareness, lidar-corrected) ──
        Node(
            package="rover_navigation",
            executable="depth_bridge_node",
            name="depth_bridge_node",
            parameters=[{
                "depth_server_url": LaunchConfiguration("depth_server_url"),
                "settle_delay": 0.5,
                "request_timeout": 0.8,
                "lidar_correction": True,
            }],
            output="screen",
        ),

        # ── TF: base_link -> laser_frame (lidar mounting offset) ───────────────
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_laser_static",
            arguments=[
                LaunchConfiguration("lidar_x"), LaunchConfiguration("lidar_y"),
                LaunchConfiguration("lidar_z"),
                LaunchConfiguration("lidar_yaw"), "0", "0",
                "base_link", "laser_frame",
            ],
        ),

        # ── TF: odom -> base_link, static identity ──────────────────────────────
        # No wheel encoders on this rover, so there's no real local motion
        # source to publish here. slam_toolbox publishes map->odom on top of
        # this and relies entirely on scan matching (no external motion prior).
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_to_base_link_static",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        ),

        # ── slam_toolbox (lidar SLAM — map->odom + occupancy grid map) ──────────
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[slam_toolbox_params],
            output="screen",
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
        # Delayed: see nav2_start_delay above.
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
