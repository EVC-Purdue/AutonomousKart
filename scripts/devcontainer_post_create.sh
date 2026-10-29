#!/usr/bin/env bash
# Build the container's python environment.
set -euo pipefail

: "${ROS_DISTRO:=humble}"
WS=/ws
VENV="$HOME/.kart-venv"

if [ -d "$VENV" ] && [ ! -f "$VENV/.hermetic" ]; then
  rm -rf "$VENV"
fi
if [ ! -d "$VENV" ]; then
  python3 -m venv "$VENV"
  touch "$VENV/.hermetic"
fi
# shellcheck disable=SC1091
source "$VENV/bin/activate"
python -m pip install --upgrade pip wheel

# One resolver pass over both, so a conflict fails loudly here.
python -m pip install -r "$WS/requirements.txt" -r "$WS/requirements-dev.txt"
if [ -f "$WS/requirements.local.txt" ]; then
  python -m pip install -r "$WS/requirements.local.txt" || true
fi

# ROS's own python.
cat > "$VENV/lib/python3.10/site-packages/ros2.pth" <<PTH
/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages
/opt/ros/${ROS_DISTRO}/local/lib/python3.10/dist-packages
PTH

rosdep install --from-paths "$WS/src" --ignore-src -y || true
