#!/bin/bash
set -euo pipefail

ROVER_HOST="raspberrypi.local"
ROVER_USER="konkon"
ROVER_WS="/home/konkon/dev/autonomous-rover"
BRANCH=$(git rev-parse --abbrev-ref HEAD)

echo "Deploying branch '$BRANCH' to $ROVER_USER@$ROVER_HOST..."

# Push current branch to GitHub first
git push origin "$BRANCH"

REPO_URL=$(git remote get-url origin)

# SSH into Pi: clean clone and rebuild
ssh "$ROVER_USER@$ROVER_HOST" bash <<EOF
set -eo pipefail
rm -rf "$ROVER_WS"
git clone --branch $BRANCH $REPO_URL $ROVER_WS
cd $ROVER_WS
source /opt/ros/humble/setup.bash
colcon build --symlink-install
echo "Build complete."
EOF

echo "Deploy done. SSH in with: ssh $ROVER_USER@$ROVER_HOST"
