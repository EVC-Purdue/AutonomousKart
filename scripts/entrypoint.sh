#!/usr/bin/env bash
set -e
# Source ROS
source /opt/ros/$ROS_DISTRO/setup.bash || true
# Source the workspace if it exists
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
