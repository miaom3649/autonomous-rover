import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
            output="screen",
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

        # ── map→odom: static identity — ORB-SLAM3 is the only position source ──
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_static",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
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
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
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
