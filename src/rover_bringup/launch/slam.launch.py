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
    slam_toolbox_params = os.path.join(
        home, "dev", "autonomous-rover", "config", "slam_toolbox_params.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false"),

        # drive_node owns the Picarx instance and also publishes ultrasonic range
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
            parameters=[{"use_sim": use_sim, "frame_width": 320, "frame_height": 240}],
        ),

        # Convert ultrasonic Range → LaserScan for slam_toolbox
        Node(
            package="rover_navigation",
            executable="ultrasonic_to_scan_node",
            name="ultrasonic_to_scan_node",
        ),

        # ORB-SLAM3 for camera-based localization
        # output='screen' is required so ORB-SLAM3's internal std::cout tracking
        # messages appear in the launch output instead of going only to ~/.ros/log/
        Node(
            package="rover_slam",
            executable="orb_slam3_node",
            name="orb_slam3_node",
            output="screen",
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

        # slam_toolbox for 2D occupancy map building
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[slam_toolbox_params],
            remappings=[("pose", "/slam_toolbox/pose")],
            ros_arguments=["--log-level", "WARN"],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_tf",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ),
    ])
