#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/.venv/bin/activate ] && source /ws/.venv/bin/activate

colcon build
ros2 launch autonomous_kart bringup_sim.launch.py