import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim = LaunchConfiguration("use_sim")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false"),

        Node(
            package="rover_base",
            executable="drive_node",
            name="drive_node",
            parameters=[{"use_sim": use_sim}],
        ),
        Node(
            package="rover_control",
            executable="mode_controller_node",
            name="mode_controller_node",
        ),
        # Remap teleop_twist_keyboard output → /rover/cmd_vel_teleop
        Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            name="teleop_twist_keyboard",
            remappings=[("/cmd_vel", "/rover/cmd_vel_teleop")],
            output="screen",
        ),
    ])
