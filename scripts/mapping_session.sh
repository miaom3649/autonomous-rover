#!/bin/bash
# Run on the Pi to do a full mapping session.
# Usage: bash scripts/mapping_session.sh
set -eo pipefail

source /opt/ros/humble/setup.bash
source "$HOME/dev/autonomous-rover/install/setup.bash"

SLAM_PID=""
MONITOR_PID=""

cleanup() {
    echo ""
    echo "Stopping..."
    [ -n "$MONITOR_PID" ] && kill "$MONITOR_PID" 2>/dev/null || true
    [ -n "$SLAM_PID" ]   && kill "$SLAM_PID"    2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT

# ── 1. Start SLAM ──────────────────────────────────────────────────────────
echo "Starting SLAM stack..."
ros2 launch rover_bringup slam.launch.py &
SLAM_PID=$!
sleep 6  # wait for nodes to come up

# ── 2. Background monitor — prints when SLAM initialises ──────────────────
(
    while ! timeout 3 ros2 topic echo --once /orb_slam3/pose &>/dev/null; do
        sleep 2
    done
    echo ""
    echo "========================================="
    echo "  SLAM INITIALISED — keep driving to map"
    echo "========================================="
    echo ""
) &
MONITOR_PID=$!

# ── 3. Teleop in foreground (needs real TTY) ───────────────────────────────
echo ""
echo "Move the rover to initialise SLAM, then keep driving to map the area."
echo "Press Ctrl-C when done mapping."
echo ""
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap cmd_vel:=/rover/cmd_vel

# ── 4. Save map ───────────────────────────────────────────────────────────
echo "Saving map..."
mkdir -p "$HOME/maps"
ros2 run nav2_map_server map_saver_cli -f "$HOME/maps/room"
echo "Done — map saved to ~/maps/room.pgm and ~/maps/room.yaml"
