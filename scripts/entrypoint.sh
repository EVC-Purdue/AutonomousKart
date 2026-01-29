#!/usr/bin/env bash
set -e
# Source ROS
source /opt/ros/humble/setup.bash
colcon build
# Source the workspace if it exists
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"

if [ -f /ws/install/setup.bash ]; then
    sed -i 's/\r$//' /ws/install/setup.bash
fi
if [ -f /ws/install/local_setup.bash ]; then
    sed -i 's/\r$//' /ws/install/local_setup.bash
fi
