#!/bin/bash
set -euo pipefail

ROVER_HOST="raspberrypi.local"
ROVER_USER="konkon"
ROVER_WS="/home/konkon/dev/autonomous-rover"
BRANCH=$(git rev-parse --abbrev-ref HEAD)

echo "Deploying branch '$BRANCH' to $ROVER_USER@$ROVER_HOST..."

git push origin "$BRANCH"

REPO_URL=$(git remote get-url origin)

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

# Check if rover_slam C++ source changed since last successful build
SLAM_STAMP="$ROVER_WS/build/rover_slam/.last_built_commit"
SLAM_CURRENT=\$(git rev-parse HEAD -- src/rover_slam 2>/dev/null || echo "unknown")
SLAM_LAST=\$(cat "\$SLAM_STAMP" 2>/dev/null || echo "")

if [ "\$SLAM_CURRENT" != "\$SLAM_LAST" ]; then
  echo "rover_slam changed — rebuilding C++ (takes ~2 min)..."
  colcon build --symlink-install \
    --packages-select rover_slam --parallel-workers 1 \
    --cmake-args -DORB_SLAM3_ROOT_DIR=\$HOME/ORB_SLAM3 -DPangolin_DIR=\$HOME/Pangolin/build
  mkdir -p "$ROVER_WS/build/rover_slam"
  echo "\$SLAM_CURRENT" > "\$SLAM_STAMP"
else
  echo "rover_slam unchanged — skipping C++ build."
fi

# Python packages are always fast
colcon build --symlink-install --packages-skip rover_slam

echo "Build complete."
EOF

echo "Deploy done. SSH in with: ssh $ROVER_USER@$ROVER_HOST"
