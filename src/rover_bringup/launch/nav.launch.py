import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim = LaunchConfiguration("use_sim")
    map_file = LaunchConfiguration("map_file")

    home = os.path.expanduser("~")
    vocab_path = os.path.join(home, "ORB_SLAM3", "Vocabulary", "ORBvoc.txt")
    settings_path = os.path.join(home, "dev", "autonomous-rover", "config", "orbslam3.yaml")
    base_params = os.path.join(home, "dev", "autonomous-rover", "config", "base_params.yaml")
    nav2_params = os.path.join(home, "dev", "autonomous-rover", "config", "nav2_params.yaml")
    slam_toolbox_params = os.path.join(
        home, "dev", "autonomous-rover", "config", "slam_toolbox_params.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false",
                              description="Run in simulation mode (no hardware required)"),
        DeclareLaunchArgument(
            "map_file",
            default_value=os.path.join(home, "maps", "room"),
            description="Path to serialized slam_toolbox map (without extension)"),

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
        Node(
            package="rover_base",
            executable="ultrasonic_sensor_node",
            name="ultrasonic_sensor_node",
            parameters=[base_params, {"use_sim": use_sim}],
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
        ),
        Node(
            package="rover_navigation",
            executable="slam_pose_bridge",
            name="slam_pose_bridge",
        ),

        # ── Ultrasonic → LaserScan ────────────────────────────────────────────
        Node(
            package="rover_navigation",
            executable="ultrasonic_to_scan_node",
            name="ultrasonic_to_scan_node",
        ),

        # ── slam_toolbox localization: loads saved map, publishes /map + TF ───
        Node(
            package="slam_toolbox",
            executable="localization_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[slam_toolbox_params, {
                "mode": "localization",
                "map_file_name": map_file,
            }],
            output="screen",
        ),

        # ── Mode controller (MANUAL/AUTO arbitration + estop) ────────────────
        Node(
            package="rover_control",
            executable="mode_controller_node",
            name="mode_controller_node",
        ),

        # ── Nav2 stack ────────────────────────────────────────────────────────
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
            package="nav2_recoveries",
            executable="recoveries_server",
            name="recoveries_server",
            parameters=[nav2_params],
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
    ])
