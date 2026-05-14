#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/.venv/bin/activate ] && source /ws/.venv/bin/activate

colcon build
pkill -f master_api || true

# Lat/long are approx west lafayette, not important to be highly accurate
mkdir -p /ws/logs
str2str -in ntrip://shayman1:shayman1@108.59.49.226:9000/MSM4_VRS -out tcpsvr://:9195 -b 1 -p 40.4376975222 -86.9444409756 200 >> /ws/logs/str2str.log 2>&1 &

sleep infinity