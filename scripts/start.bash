#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/.venv/bin/activate ] && source /ws/.venv/bin/activate

colcon build
pkill -f master_api || true

str2str -in ntrip://shayman1:shayman1@108.59.49.226:9000/MSM4_VRS -out tcpsvr://:9195 -b 1 &
mkdir -p /ws/logs

# Logs supervisor - crash tolerant
(
  while true; do
    RUN_NAME="run_$(date -u +%Y%m%d_%H%M%S)"
    ros2 bag record -a \
      --storage mcap \
      --storage-preset-profile zstd_fast \
      --max-bag-duration 60 \
      --exclude '/camera/image_raw' \
      --output "/ws/logs/$RUN_NAME" \
      >> /ws/logs/recorder.log 2>&1
    echo "[$(date -u +%FT%TZ)] recorder exited, restarting in 1s" >> /ws/logs/recorder.log
    sleep 1
  done
) &

sleep infinity