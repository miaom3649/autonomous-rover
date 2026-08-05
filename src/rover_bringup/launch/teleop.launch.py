import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim = LaunchConfiguration("use_sim")

    home = os.path.expanduser("~")
    base_params = os.path.join(home, "dev", "autonomous-rover", "config", "base_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false"),

        Node(
            package="rover_base",
            executable="drive_node",
            name="drive_node",
            parameters=[base_params, {"use_sim": use_sim}],
        ),
        Node(
            package="rover_control",
            executable="mode_controller_node",
            name="mode_controller_node",
            parameters=[base_params],
        ),
        # Remap teleop_twist_keyboard output → /rover/cmd_vel_teleop
    ])
