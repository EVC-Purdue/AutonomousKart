#!/usr/bin/env bash
set -euo pipefail

: "${ROS_DISTRO:=humble}"

# Source ROS and prep workspace
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source "/opt/ros/humble/setup.bash"
fi

if [ ! -d /ws/.venv ]; then
  python3 -m venv /ws/.venv
fi
source /ws/.venv/bin/activate
python -m pip install --upgrade pip setuptools wheel

if [ -f /ws/requirements.txt ]; then
  pip install -r /ws/requirements.txt
fi
if [ -f /ws/requirements.local.txt ]; then
  pip install -r /ws/requirements.local.txt || true
fi

rosdep update || true
rosdep install --from-paths /ws/src --ignore-src -y || true

colcon build --symlink-install || true
