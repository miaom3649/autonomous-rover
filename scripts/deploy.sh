#!/bin/bash
set -euo pipefail

ROVER_HOST="raspberrypi.local"
ROVER_USER="konkon"
ROVER_WS="/home/konkon/dev/autonomous-rover"
BRANCH=$(git rev-parse --abbrev-ref HEAD)

echo "Deploying branch '$BRANCH' to $ROVER_USER@$ROVER_HOST..."

# Push current branch to GitHub first
git push origin "$BRANCH"

# SSH into Pi: pull latest code and rebuild
ssh "$ROVER_USER@$ROVER_HOST" bash <<EOF
set -eo pipefail
cd $ROVER_WS
rm -rf *
git fetch origin
git checkout $BRANCH
git pull origin $BRANCH
source /opt/ros/humble/setup.bash
colcon build --symlink-install
echo "Build complete."
EOF

echo "Deploy done. SSH in with: ssh $ROVER_USER@$ROVER_HOST"
