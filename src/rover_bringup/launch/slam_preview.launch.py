"""
Minimal SLAM preview — camera + orb_slam3_node only, no driving, no Nav2.

For hand-carrying the rover around to eyeball ORB-SLAM3's tracking/mapping
quality before trusting it for actual navigation. Open
http://raspberrypi.local:8082 to see the live RGB feed, corrected depth, and
ORB-SLAM3's own debug view (tracked keypoints, green = tracking OK, red =
lost) side by side.

depth_bridge_node is still required even for this minimal test — orb_slam3_node
runs in RGBD mode and needs a depth map alongside each RGB frame, not just the
camera. ultrasonic_correction is off here since the rover isn't being driven
on the ground (nothing meaningful for the ultrasonic sensor to anchor scale
to while hand-held) — depth is used uncorrected, which is fine for eyeballing
tracking/mapping quality, just not for metric accuracy.

Unlike nav.launch.py's stop-and-go rhythm (driven by the AI depth round trip
being too slow to run at camera framerate while also planning/driving),
there's no cmd_vel being published at all here (no drive_node, no
mode_controller_node) — depth_bridge_node's stop-detection logic treats
"never received a moving cmd_vel" as "always stopped," so it fires as fast as
its own processing loop allows, giving a roughly continuous feed while
walking. camera_node's publish_rate_hz is bumped up accordingly, since
nav.launch.py's 2.0 Hz was specifically tuned for one-frame-per-stop, not for
smooth continuous tracking while moving.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    home = os.path.expanduser("~")
    vocab_path = os.path.join(home, "ORB_SLAM3", "Vocabulary", "ORBvoc.txt")
    settings_path = os.path.join(home, "dev", "autonomous-rover", "config", "orbslam3.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "depth_server_url",
                default_value="http://192.168.1.151:8765/depth",
                description="URL of the Windows depth inference server",
            ),
            Node(
                package="rover_camera",
                executable="camera_node",
                name="camera_node",
                parameters=[
                    {
                        "use_sim": False,
                        "frame_width": 320,
                        "frame_height": 240,
                        "publish_rate_hz": 10.0,
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
                        "settle_delay": 0.5,
                        "request_timeout": 0.8,
                        "ultrasonic_correction": False,
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
