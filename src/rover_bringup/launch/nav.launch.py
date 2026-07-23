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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim",
                default_value="false",
                description="Run in simulation mode (no hardware required)",
            ),
            DeclareLaunchArgument(
                "dashboard_start_delay",
                default_value="5.0",
                description="Seconds to wait before starting dashboard_node, staggering it "
                "slightly behind the rest of the startup rather than starting everything "
                "at t=0 at once.",
            ),
            DeclareLaunchArgument(
                "nav2_start_delay",
                default_value="15.0",
                description=(
                    "Seconds to wait before starting the Nav2 stack. Lower than the old "
                    "ORB-SLAM3-era value (45.0) — slam_toolbox has no large vocabulary "
                    "file to load, so the startup memory-pressure risk that justified "
                    "the longer delay doesn't apply the same way here."
                ),
            ),
            DeclareLaunchArgument(
                "lidar_x",
                default_value="0.0",
                description="Lidar x offset from base_link, meters — PLACEHOLDER, measure "
                "once the lidar is permanently mounted.",
            ),
            DeclareLaunchArgument(
                "lidar_y",
                default_value="0.0",
                description="Lidar y offset from base_link, meters — PLACEHOLDER.",
            ),
            DeclareLaunchArgument(
                "lidar_z",
                default_value="0.05",
                description="Lidar z offset from base_link, meters — PLACEHOLDER.",
            ),
            DeclareLaunchArgument(
                "lidar_yaw",
                default_value="0.0",
                description="Lidar yaw offset from base_link, radians — PLACEHOLDER.",
            ),
            # ── Hardware nodes ──
            Node(
                package="rover_base",
                executable="drive_node",
                name="drive_node",
                parameters=[base_params, {"use_sim": use_sim}],
            ),
            # ── Lidar (YDLidar X3, primary localization/mapping sensor) ──
            Node(
                package="ydlidar_ros2_driver",
                executable="ydlidar_ros2_driver_node",
                name="ydlidar_ros2_driver_node",
                parameters=[lidar_params],
                output="screen",
            ),
            # ── slam_toolbox (mapping mode, pure scan-matching — no wheel odometry) ──
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[slam_toolbox_params],
                output="screen",
            ),
            # ── TF: base_link -> laser_frame, static (measure once mounted) ──
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_laser_static",
                arguments=[
                    LaunchConfiguration("lidar_x"),
                    LaunchConfiguration("lidar_y"),
                    LaunchConfiguration("lidar_z"),
                    LaunchConfiguration("lidar_yaw"),
                    "0",
                    "0",
                    "base_link",
                    "laser_frame",
                ],
            ),
            # ── TF: odom -> base_link, static identity — no wheel encoders on this
            # rover, so slam_toolbox gets no external motion prior and relies entirely
            # on scan-to-scan / scan-to-map matching (map -> odom comes from slam_toolbox).
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_base_link_static",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            ),
            # ── Mode controller (MANUAL/AUTO arbitration + estop) ──
            Node(
                package="rover_control",
                executable="mode_controller_node",
                name="mode_controller_node",
            ),
            # ── Live debug dashboard (lidar/mode/ultrasonic/position on :8082) ──
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
            # ── Nav2 stack ──
            # Delayed: see nav2_start_delay above. Output remapped straight to
            # /rover/cmd_vel_nav — lidar gives continuous, fast localization (no AI
            # round trip to wait on), so there's no need for the old burst/pause
            # filter that used to sit between Nav2 and mode_controller_node.
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
        ]
    )
