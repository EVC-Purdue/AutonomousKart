#!/usr/bin/env bash
set -euo pipefail

: "${ROS_DISTRO:=humble}"

grep -qxF "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc || \
  echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

grep -qxF "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" ~/.bashrc || \
  echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> ~/.bashrc

grep -qxF "[ -f /ws/.venv/bin/activate ] && source /ws/.venv/bin/activate" ~/.bashrc || \
  echo "[ -f /ws/.venv/bin/activate ] && source /ws/.venv/bin/activate" >> ~/.bashrc
