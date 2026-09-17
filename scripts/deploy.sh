#!/bin/bash
set -euo pipefail

ROVER_HOST="kk.local"
ROVER_USER="kk"
ROVER_WS="/home/kk/dev/autonomous-rover"
REPO_URL="https://github.com/miaom3649/autonomous-rover.git"
BRANCH=$(git rev-parse --abbrev-ref HEAD)

echo "Deploying branch '$BRANCH' to $ROVER_USER@$ROVER_HOST..."

git push origin "$BRANCH"

ssh "$ROVER_USER@$ROVER_HOST" bash <<EOF
set -eo pipefail
source /opt/ros/humble/setup.bash

# First deploy: clone the repo if workspace doesn't exist yet
if [ ! -d "$ROVER_WS/.git" ]; then
  echo "Fresh install — cloning repo..."
  git clone --branch $BRANCH $REPO_URL $ROVER_WS
fi

# Ensure ROS tools are installed
dpkg -s ros-humble-teleop-twist-keyboard &>/dev/null || \
  sudo apt-get install -y ros-humble-teleop-twist-keyboard

cd $ROVER_WS
git fetch origin
git checkout $BRANCH
git reset --hard origin/$BRANCH

# Clean the project's Python package build directories so stale installed
# modules do not shadow the current source. The vendored lidar/rf2o C++
# packages are left incremental because rebuilding them is expensive on Pi 4.
rm -rf build/rover_base build/rover_navigation build/rover_bringup build/rover_control
# Cap compiler jobs per package too — on the 2GB Pi 4, letting a single
# template-heavy package (e.g. rf2o_laser_odometry) spawn one compiler
# process per core exhausts RAM and swap-thrashes the system instead of
# actually building faster.
export MAKEFLAGS="-j1"
# ydlidar_ros2_driver depends on the YDLidar-SDK, which isn't a ROS/apt
# package — it's built from source into ~/.local (see README in
# src/ydlidar_ros2_driver) so find_package(ydlidar_sdk) can locate it.
export CMAKE_PREFIX_PATH="\$HOME/.local:\${CMAKE_PREFIX_PATH:-}"
colcon build --symlink-install --parallel-workers 1 --event-handlers console_direct+

mkdir -p /home/kk/maps

echo "Build complete."
EOF

echo "Deploy done. SSH in with: ssh $ROVER_USER@$ROVER_HOST"
