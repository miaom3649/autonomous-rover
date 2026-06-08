import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim = LaunchConfiguration("use_sim")
    map_yaml = LaunchConfiguration("map")

    home = os.path.expanduser("~")
    vocab_path = os.path.join(home, "ORB_SLAM3", "Vocabulary", "ORBvoc.txt")
    settings_path = os.path.join(home, "dev", "autonomous-rover", "config", "orbslam3.yaml")
    nav2_params = os.path.join(home, "dev", "autonomous-rover", "config", "nav2_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false",
                              description="Run in simulation mode (no hardware required)"),
        DeclareLaunchArgument("map", default_value=os.path.join(home, "maps", "room.yaml"),
                              description="Path to the 2D occupancy grid map YAML file"),

        # ── Hardware nodes ────────────────────────────────────────────────────
        Node(
            package="rover_base",
            executable="camera_node",
            name="camera_node",
            parameters=[{"use_sim": use_sim}],
        ),
        Node(
            package="rover_base",
            executable="ultrasonic_sensor_node",
            name="ultrasonic_sensor_node",
            parameters=[{"use_sim": use_sim}],
        ),

        # ── SLAM nodes ────────────────────────────────────────────────────────
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
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_tf",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ),

        # ── Nav2 nodes ────────────────────────────────────────────────────────
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[nav2_params, {"yaml_filename": map_yaml}],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            parameters=[nav2_params],
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
