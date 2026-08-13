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
    slam_localization_params = os.path.join(
        home, "dev", "autonomous-rover", "config", "slam_toolbox_localization_params.yaml"
    )
    rf2o_params = os.path.join(home, "dev", "autonomous-rover", "config", "rf2o_params.yaml")
    camera_projection_params = os.path.join(
        home, "dev", "autonomous-rover", "config", "camera_projection_params.yaml"
    )
    object_detection_params = os.path.join(
        home, "dev", "autonomous-rover", "config", "object_detection_params.yaml"
    )
    mapping_monitor_params = os.path.join(
        home, "dev", "autonomous-rover", "config", "mapping_monitor_params.yaml"
    )
    semantic_mapping_params = os.path.join(
        home, "dev", "autonomous-rover", "config", "semantic_mapping_params.yaml"
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
                default_value="0.130900",
                description="Lidar yaw offset from base_link, radians (calibrated +7.5 deg).",
            ),
            # ── Hardware nodes ──
            Node(
                package="rover_base",
                executable="drive_node",
                name="drive_node",
                parameters=[base_params, {"use_sim": use_sim}],
            ),
            Node(
                package="rover_base",
                executable="camera_node",
                name="camera_node",
                parameters=[base_params, {"use_sim": use_sim}],
            ),
            Node(
                package="rover_navigation",
                executable="object_detector_node",
                name="object_detector_node",
                parameters=[object_detection_params],
            ),
            Node(
                package="rover_navigation",
                executable="mapping_monitor_node",
                name="mapping_monitor_node",
                parameters=[mapping_monitor_params, {
                    "slam_params_file": slam_toolbox_params,
                    "slam_localization_params_file": slam_localization_params,
                }],
            ),
            Node(
                package="rover_navigation",
                executable="object_localizer_node",
                name="object_localizer_node",
                parameters=[semantic_mapping_params],
            ),
            Node(
                package="rover_navigation",
                executable="semantic_mapper_node",
                name="semantic_mapper_node",
                parameters=[semantic_mapping_params],
            ),
            Node(
                package="rover_navigation",
                executable="semantic_navigation_node",
                name="semantic_navigation_node",
                parameters=[semantic_mapping_params],
            ),
            # ── Lidar (YDLidar X3, primary localization/mapping sensor) ──
            Node(
                package="ydlidar_ros2_driver",
                executable="ydlidar_ros2_driver_node",
                name="ydlidar_ros2_driver_node",
                parameters=[lidar_params],
                output="screen",
            ),
            # ── Laser odometry (rf2o) — this rover has no wheel encoders, so this
            # estimates odom -> base_link purely from consecutive /scan frames and
            # publishes both /rover/odom and the TF, replacing the old static-
            # identity placeholder. Gives slam_toolbox a real motion prior to
            # narrow its scan-matching search around instead of searching blind.
            Node(
                package="rf2o_laser_odometry",
                executable="rf2o_laser_odometry_node",
                name="rf2o_laser_odometry",
                parameters=[rf2o_params],
                arguments=["--ros-args", "--log-level", "error"],
                output="screen",
            ),
            # slam_toolbox is started and switched between mapping/localization
            # modes by mapping_monitor_node. It must not be independently
            # respawned here, otherwise the mapping process would come back while
            # the fixed-map localization process is running.
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
            # odom -> base_link is now published dynamically by rf2o_laser_odometry
            # above, not a static identity transform.
            # ── Mode controller (MANUAL/AUTO arbitration + estop) ──
            Node(
                package="rover_control",
                executable="mode_controller_node",
                name="mode_controller_node",
                parameters=[base_params],
            ),
            # ── Live debug dashboard (lidar/mode/ultrasonic/position on :8082) ──
            TimerAction(
                period=LaunchConfiguration("dashboard_start_delay"),
                actions=[
                    Node(
                        package="rover_navigation",
                        executable="dashboard_node",
                        name="dashboard_node",
                        parameters=[camera_projection_params],
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
