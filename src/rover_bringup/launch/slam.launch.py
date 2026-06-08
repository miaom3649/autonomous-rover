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

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false",
                              description="Run in simulation mode (no hardware required)"),

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
        # Static identity transform: map → odom (ORB-SLAM3 provides absolute pose,
        # so odom and map share the same origin)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_tf",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ),
    ])
