#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/.venv/bin/activate ] && source /ws/.venv/bin/activate

colcon build
pkill -f master_api || true

str2str -in ntrip://shayman1:shayman1@108.59.49.226:9000/MSM4_VRS -out tcpsvr://:9195 -b 1 &
sleep infinity