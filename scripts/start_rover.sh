#!/bin/bash
# Starts all rover nodes in a tmux session with three panes.
# Usage: ./scripts/start_rover.sh [--sim]

set -euo pipefail

SESSION="rover"
WS="/home/konkon/dev/autonomous-rover"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS/install/setup.bash"
USE_SIM="false"

if [[ "${1:-}" == "--sim" ]]; then
    USE_SIM="true"
fi

if ! command -v tmux &>/dev/null; then
    echo "Error: tmux not installed. Run: sudo apt install tmux"
    exit 1
fi

if [[ ! -f "$WS_SETUP" ]]; then
    echo "Error: workspace not built. Run: colcon build inside $WS"
    exit 1
fi

SOURCE="source $ROS_SETUP && source $WS_SETUP"

# Kill any previous rover session and leftover node processes.
tmux kill-session -t "$SESSION" 2>/dev/null || true
pkill -f "ultrasonic_sensor_node" 2>/dev/null || true
pkill -f "position_estimator_node" 2>/dev/null || true
sleep 1

# Create detached tmux session.
tmux new-session -d -s "$SESSION" -x 220 -y 50

# ── Pane 1 (left): ultrasonic driver ──────────────────────────────────────────
tmux send-keys -t "$SESSION" \
    "$SOURCE && ros2 run rover_base ultrasonic_sensor_node --ros-args -p use_sim:=$USE_SIM" \
    Enter

# ── Pane 2 (top-right): position estimator ────────────────────────────────────
tmux split-window -h -t "$SESSION"
tmux send-keys -t "$SESSION" \
    "$SOURCE && ros2 run rover_navigation position_estimator_node" \
    Enter

# ── Pane 3 (bottom-right): lifecycle activation + topic monitor ───────────────
tmux split-window -v -t "$SESSION"
tmux send-keys -t "$SESSION" \
    "$SOURCE && sleep 4 \
     && ros2 lifecycle set /ultrasonic_sensor_node configure \
     && ros2 lifecycle set /ultrasonic_sensor_node activate \
     && echo '--- Kalman estimated position (m) ---' \
     && ros2 topic echo /rover/estimated_position" \
    Enter

# Focus on the monitor pane and attach.
tmux select-pane -t "$SESSION:0.2"
tmux attach-session -t "$SESSION"
