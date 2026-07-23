"""
Teleop SLAM test — manual driving with continuous (non-stop-gated) tracking.

Unlike nav.launch.py's stop-and-go rhythm (camera_node/depth_bridge_node only
publish once the rover has stopped and settled) and slam_preview.launch.py
(hand-carried, never driven), this launch drives the rover under manual
teleop control while camera_node and depth_bridge_node run in continuous
mode — publishing/fetching as fast as they can regardless of whether the
rover is moving, instead of waiting for a stop.

camera_node.continuous_publish:=true removes its stop-gate; the achievable
depth rate is then bounded by the Windows depth server's round trip (measured
~500ms, so ~2Hz), not by camera_node itself — depth_bridge_node's
continuous_mode:=true removes its own independent stop-gate but still only
ever has one request in flight at a time, so this is the real ceiling on how
often orb_slam3_node gets a synced RGBD pair while driving.

To drive, run teleop_twist_keyboard separately with its output remapped:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r cmd_vel:=/rover/cmd_vel_teleop
mode_controller_node starts in MANUAL by default, so no /rover/mode
publish is needed. Open http://raspberrypi.local:8082 to watch tracking
live while driving.
"""

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

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="false"),
            DeclareLaunchArgument(
                "depth_server_url",
                default_value="http://192.168.3.33:8765/depth",
                description="URL of the Windows depth inference server",
            ),
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
            Node(
                package="rover_camera",
                executable="camera_node",
                name="camera_node",
                parameters=[
                    {
                        "use_sim": use_sim,
                        "frame_width": 320,
                        "frame_height": 240,
                        "publish_rate_hz": 10.0,
                        "continuous_publish": True,
                    }
                ],
            ),
            Node(
                package="rover_navigation",
                executable="depth_bridge_node",
                name="depth_bridge_node",
                parameters=[
                    {
                        "depth_server_url": LaunchConfiguration("depth_server_url"),
                        "request_timeout": 0.8,
                        "ultrasonic_correction": True,
                        "continuous_mode": True,
                    }
                ],
                output="screen",
            ),
            Node(
                package="rover_slam",
                executable="orb_slam3_node",
                name="orb_slam3_node",
                parameters=[
                    {
                        "vocab_path": vocab_path,
                        "settings_path": settings_path,
                    }
                ],
                output="screen",
                sigterm_timeout="3",
                sigkill_timeout="3",
            ),
            Node(
                package="rover_navigation",
                executable="dashboard_node",
                name="dashboard_node",
            ),
        ]
    )
