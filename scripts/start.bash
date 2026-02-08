#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
colcon build --symlink-install
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/.venv/bin/activate ] && source /ws/.venv/bin/activate

# iox-roudi -c /etc/iceoryx/roudi_config.toml &
ros2 launch autonomous_kart bringup_sim.launch.py