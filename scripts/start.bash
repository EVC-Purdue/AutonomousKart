#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/.venv/bin/activate ] && source /ws/.venv/bin/activate

colcon build
pkill -f master_api || true

# Lat/long are approx west lafayette, not important to be highly accurate
mkdir -p /ws/logs
str2str -in ntrip://shayman1:shayman1@108.59.49.226:9000/MSM4_VRS -out tcpsvr://:9195 -b 1 -p 40.4376975222 -86.9444409756 200 >> /ws/logs/str2str.log 2>&1 &
pip install joblib
# Bag recorder records only while driving (AUTONOMOUS/MANUAL)
# Polls the current state instead of watching for transitions, so a missed edge
# self-corrects on the next poll. An unreadable state leaves recording as-is so
# an API hiccup never drops a bag.
BAG_SPLIT_S=10
BAG_CACHE_BYTES=1048576
BAG_EXCLUDE='/camera/.*'
REC_PID=""
REC_DIR=""

kgit() { git -c safe.directory=/ws -C /ws "$@"; }

# Everything needed to reproduce the run: commit, uncommitted work, params.
snapshot_run() {
  local run_dir="$1" state="$2"
  local commit branch describe dirty

  commit="$(kgit rev-parse HEAD 2>/dev/null || echo unknown)"
  branch="$(kgit rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
  describe="$(kgit describe --always --dirty --tags 2>/dev/null || echo unknown)"
  if kgit diff --quiet HEAD 2>/dev/null; then
    dirty=false
  else
    dirty=true
    kgit diff HEAD > "$run_dir/uncommitted.patch" 2>/dev/null
  fi

  # The yamls the launch actually loaded (install tree), source tree as fallback.
  mkdir -p "$run_dir/params"
  cp /ws/install/autonomous_kart/share/autonomous_kart/params/*.yaml "$run_dir/params/" 2>/dev/null \
    || cp /ws/src/autonomous_kart/autonomous_kart/params/*.yaml "$run_dir/params/" 2>/dev/null

  cat > "$run_dir/manifest.json" <<EOF
{
  "started_at": "$(date -u +%FT%TZ)",
  "trigger_state": "$state",
  "commit": "$commit",
  "branch": "$branch",
  "describe": "$describe",
  "dirty": $dirty,
  "host": "$(hostname)",
  "ros_distro": "${ROS_DISTRO:-unknown}",
  "record_flags": "-a --storage mcap --storage-preset-profile zstd_fast --max-bag-duration $BAG_SPLIT_S --max-cache-size $BAG_CACHE_BYTES --exclude '$BAG_EXCLUDE'"
}
EOF

  # Effective param values, after the /**: wildcards and launch overrides.
  # Run from the run dir so a dump that writes files lands here, not in /ws.
  (
    cd "$run_dir" || exit 0
    for node in $(ros2 node list 2>/dev/null); do
      echo "# $node"
      ros2 param dump "$node" 2>/dev/null
    done > params_live.yaml
  )
}

start_run() {
  local state="$1" tag
  case "$state" in
    AUTONOMOUS) tag=auto ;;
    *) tag=manual ;;
  esac
  REC_DIR="/ws/logs/run_$(date -u +%Y%m%d_%H%M%S)_$tag"
  mkdir -p "$REC_DIR"
  # bash hands a background job an ignored SIGINT, which rosbag2 would then never
  # see. Reset the disposition and exec in place, so $REC_PID is the recorder
  # itself and a stop reaches it.
  python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
    ros2 bag record -a \
    --storage mcap \
    --storage-preset-profile zstd_fast \
    --max-bag-duration "$BAG_SPLIT_S" \
    --max-cache-size "$BAG_CACHE_BYTES" \
    --exclude "$BAG_EXCLUDE" \
    --output "$REC_DIR/bag" \
    >> /ws/logs/recorder.log 2>&1 &
  REC_PID=$!
  snapshot_run "$REC_DIR" "$state" >> /ws/logs/recorder.log 2>&1 &
  echo "[$(date -u +%FT%TZ)] recording $REC_DIR (state=$state)" >> /ws/logs/recorder.log
}

# SIGINT lets rosbag2 close the split and write metadata.yaml before we give up.
stop_run() {
  [ -z "$REC_PID" ] && return
  kill -INT "$REC_PID" 2>/dev/null
  for _ in $(seq 50); do
    kill -0 "$REC_PID" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL "$REC_PID" 2>/dev/null
  wait "$REC_PID" 2>/dev/null
  echo "[$(date -u +%FT%TZ)] stopped $REC_DIR" >> /ws/logs/recorder.log
  REC_PID=""
  REC_DIR=""
}

# Gate loop runs in the foreground so a container stop (SIGTERM) closes the bag
# cleanly instead of killing the script out from under an open split.
trap 'stop_run; exit 0' TERM INT
echo "[$(date -u +%FT%TZ)] state-gated recorder armed" >> /ws/logs/recorder.log

while true; do
  STATE="$(curl -s --max-time 1 http://127.0.0.1:8000/get_state \
    | sed -n 's/.*"state": *"\([A-Z]*\)".*/\1/p')"
  case "$STATE" in
    AUTONOMOUS|MANUAL)
      if [ -z "$REC_PID" ] || ! kill -0 "$REC_PID" 2>/dev/null; then
        [ -n "$REC_PID" ] && echo "[$(date -u +%FT%TZ)] recorder died, restarting" >> /ws/logs/recorder.log
        start_run "$STATE"
      fi
      ;;
    IDLE|STOPPED)
      stop_run
      ;;
  esac
  sleep 0.25
done
