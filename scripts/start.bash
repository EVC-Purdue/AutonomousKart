#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/.venv/bin/activate ] && source /ws/.venv/bin/activate

colcon build
pkill -f master_api || true
sleep 0.5