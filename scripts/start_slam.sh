#!/bin/bash
# Start the SLAM mapping session.
# On Ctrl+C: saves the slam_toolbox map, then shuts down all nodes.
#
# Usage: bash scripts/start_slam.sh [ros2 launch args...]
set -euo pipefail

MAP_FILE="${MAP_FILE:-$HOME/maps/room}"

source /opt/ros/humble/setup.bash
source "$HOME/dev/autonomous-rover/install/setup.bash"

mkdir -p "$(dirname "$MAP_FILE")"

ros2 launch rover_bringup slam.launch.py "$@" &
LAUNCH_PID=$!

_save_and_exit() {
    echo ""
    echo "[start_slam] Saving slam_toolbox map to $MAP_FILE ..."
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
        "{filename: '$MAP_FILE'}" 2>/dev/null && echo "[start_slam] Map saved." \
        || echo "[start_slam] WARNING: map save failed (was SLAM initialised?)"
    sleep 1
    kill "$LAUNCH_PID" 2>/dev/null
    wait "$LAUNCH_PID" 2>/dev/null || true
}

trap _save_and_exit SIGINT SIGTERM
wait "$LAUNCH_PID"
