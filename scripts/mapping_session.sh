#!/bin/bash
# Run on the Pi to do a full mapping session.
# Usage: bash scripts/mapping_session.sh
set -euo pipefail

source /opt/ros/humble/setup.bash
source "$HOME/dev/autonomous-rover/install/setup.bash"

SLAM_PID=""

cleanup() {
    echo ""
    echo "Stopping SLAM stack..."
    [ -n "$SLAM_PID" ] && kill "$SLAM_PID" 2>/dev/null || true
    wait "$SLAM_PID" 2>/dev/null || true
}
trap cleanup EXIT

# ── 1. Start SLAM in background ────────────────────────────────────────────
echo "Starting SLAM stack..."
ros2 launch rover_bringup slam.launch.py &
SLAM_PID=$!

# ── 2. Wait for ORB-SLAM3 to initialise ───────────────────────────────────
echo "Waiting for SLAM to initialise — point camera at a textured surface and move slowly..."
while ! timeout 3 ros2 topic echo --once /orb_slam3/pose &>/dev/null; do
    echo "  Not yet — keep moving..."
done

echo ""
echo "SLAM initialised! Drive around to map the area."
echo "When done, press Ctrl-C to stop teleop and save the map."
echo ""

# ── 3. Teleop ─────────────────────────────────────────────────────────────
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap cmd_vel:=/rover/cmd_vel || true

# ── 4. Save map ───────────────────────────────────────────────────────────
echo ""
echo "Saving map..."
mkdir -p "$HOME/maps"
ros2 run nav2_map_server map_saver_cli -f "$HOME/maps/room"
echo "Done — map saved to ~/maps/room.pgm and ~/maps/room.yaml"
