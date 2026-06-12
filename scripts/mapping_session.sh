#!/bin/bash
# Run on the Pi to do a full mapping session.
# Usage: bash scripts/mapping_session.sh
# Requires tmux (sudo apt install tmux).

SESSION="slam"
LOG_FILE="/tmp/slam.log"

if ! command -v tmux &>/dev/null; then
    echo "tmux not found — install with: sudo apt install tmux"
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$HOME/dev/autonomous-rover/install/setup.bash"

# Kill any leftover tmux session from a previous run
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Kill stale ROS processes
echo "Cleaning up stale processes..."
pkill -f "drive_node"          2>/dev/null || true
pkill -f "orb_slam3_node"      2>/dev/null || true
pkill -f "async_slam_toolbox"  2>/dev/null || true
pkill -f "camera_node"         2>/dev/null || true
sleep 2

# ── Build the tmux session ────────────────────────────────────────────────────
# Pane 0 (top ~70%): SLAM launch output filtered to key events
# Pane 1 (bottom ~30%): teleop + SLAM initialised banner

SETUP="source /opt/ros/humble/setup.bash && source $HOME/dev/autonomous-rover/install/setup.bash"

# Pane 0: start SLAM, tee to log file, filter for key events
SLAM_CMD="$SETUP && ros2 launch rover_bringup slam.launch.py 2>&1 | tee $LOG_FILE | grep --line-buffered -iE 'map created|match|fail|track|reset|ready|error|reseting|initializ'"

# Pane 1: wait for nodes, show banner on init, then run teleop, then save map
MONITOR="(while ! timeout 3 ros2 topic echo --once /orb_slam3/pose &>/dev/null; do sleep 2; done; printf '\n\e[1;32m=========================================\n  SLAM INITIALISED — keep driving to map\n=========================================\e[0m\n\n') &"
TELEOP="ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/rover/cmd_vel"
SAVE='echo ""; echo "Saving map..."; mkdir -p ~/maps && ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: \"$HOME/maps/room\"}}" && echo "Map saved to ~/maps/room.pgm"'

TELEOP_CMD="$SETUP && sleep 8 && $MONITOR $TELEOP; $SAVE"

tmux new-session -d -s "$SESSION" -x "$(tput cols 2>/dev/null || echo 200)" -y "$(tput lines 2>/dev/null || echo 50)"
tmux send-keys -t "$SESSION:0.0" "$SLAM_CMD" Enter
tmux split-window -v -t "$SESSION:0" -p 32
tmux send-keys -t "$SESSION:0.1" "$TELEOP_CMD" Enter
tmux select-pane -t "$SESSION:0.1"  # start with focus on teleop pane

tmux attach -t "$SESSION"

# After the user detaches / closes tmux, kill everything
pkill -f "drive_node"          2>/dev/null || true
pkill -f "orb_slam3_node"      2>/dev/null || true
pkill -f "async_slam_toolbox"  2>/dev/null || true
pkill -f "camera_node"         2>/dev/null || true
