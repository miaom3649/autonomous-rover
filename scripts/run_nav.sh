#!/bin/bash
# Runs `ros2 launch rover_bringup nav.launch.py` inside a memory-capped
# user cgroup scope.
#
# Why: on the 2GB Pi 4, if the launch tree's memory demand ever exceeds
# what's available, letting it spill onto the slow SD-card-backed swapfile
# causes the whole system to swap-thrash into total unresponsiveness
# without ever triggering the kernel OOM-killer (see README "Resource-
# constrained builds" section for the full story). Capping MemoryMax/
# MemorySwapMax forces a real, bounded OOM-kill of just this scope instead,
# and confines swap usage to the fast zram device (see /etc/default/zramswap)
# rather than the slow on-disk /swapfile.
#
# Usage: ./scripts/run_nav.sh [extra ros2 launch args...]

set -euo pipefail

WS="/home/konkon/dev/autonomous-rover"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS/install/setup.bash"

MEMORY_MAX="${MEMORY_MAX:-1400M}"
MEMORY_SWAP_MAX="${MEMORY_SWAP_MAX:-1200M}"

if [[ ! -f "$WS_SETUP" ]]; then
    echo "Error: workspace not built. Run: colcon build inside $WS"
    exit 1
fi

# The YDLidar SDK itself (not our driver code, and not gated by any ROS
# parameter) prints "Real points N > fixed points M" whenever the X3's
# actual RPM drifts slightly from the nominal rate — harmless, constant
# noise, filtered here since there's nothing to fix on our side.
exec systemd-run --scope --user \
    -p MemoryMax="$MEMORY_MAX" \
    -p MemorySwapMax="$MEMORY_SWAP_MAX" \
    -- bash -c "set -o pipefail; source $ROS_SETUP && source $WS_SETUP && ros2 launch rover_bringup nav.launch.py $* 2>&1 | grep --line-buffered -Ev 'Real points [0-9]+ > fixed points [0-9]+'"
