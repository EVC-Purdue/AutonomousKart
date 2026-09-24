#!/usr/bin/env bash
# State-gated bag recorder; scripts/kart owns env, build, launch and NTRIP.
WS="${KART_WS:-${WS:-/ws}}"

mkdir -p "$WS/logs"
# Bag recorder records only while driving (AUTONOMOUS/MANUAL)
# Polls the current state instead of watching for transitions, so a missed edge
# self-corrects on the next poll. An unreadable state leaves recording as-is so
# an API hiccup never drops a bag.
BAG_SPLIT_S=10
BAG_CACHE_BYTES=1048576
BAG_EXCLUDE='/camera/.*'
REC_PID=""
REC_DIR=""
SNAP_PID=""

kgit() { git -c safe.directory="$WS" -C "$WS" "$@"; }

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
  cp "$WS"/install/autonomous_kart/share/autonomous_kart/params/*.yaml "$run_dir/params/" 2>/dev/null \
    || cp "$WS"/src/autonomous_kart/autonomous_kart/params/*.yaml "$run_dir/params/" 2>/dev/null

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
  # Run from the run dir so a dump that writes files lands here, not in $WS.
  (
    cd "$run_dir" || exit 0
    # Time-limited: a dump against a departing node hangs holding a participant.
    for node in $(timeout 10 ros2 node list 2>/dev/null); do
      echo "# $node"
      timeout 10 ros2 param dump "$node" 2>/dev/null
    done > params_live.yaml
  )
}

start_run() {
  local state="$1" tag
  case "$state" in
    AUTONOMOUS) tag=auto ;;
    *) tag=manual ;;
  esac
  REC_DIR="$WS/logs/run_$(date -u +%Y%m%d_%H%M%S)_$tag"
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
    >> "$WS/logs/recorder.log" 2>&1 &
  REC_PID=$!
  snapshot_run "$REC_DIR" "$state" >> "$WS/logs/recorder.log" 2>&1 &
  SNAP_PID=$!
  echo "[$(date -u +%FT%TZ)] recording $REC_DIR (state=$state)" >> "$WS/logs/recorder.log"
}

# snapshot_run is backgrounded, so it and its children inherit an ignored
stop_snapshot() {
  [ -z "$SNAP_PID" ] && return
  pkill -TERM -P "$SNAP_PID" 2>/dev/null
  kill -TERM "$SNAP_PID" 2>/dev/null
  SNAP_PID=""
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
  echo "[$(date -u +%FT%TZ)] stopped $REC_DIR" >> "$WS/logs/recorder.log"
  REC_PID=""
  REC_DIR=""
}

# Gate loop runs in the foreground so a container stop (SIGTERM) closes the bag
# cleanly instead of killing the script out from under an open split.
trap 'stop_run; stop_snapshot; exit 0' TERM INT
echo "[$(date -u +%FT%TZ)] state-gated recorder armed" >> "$WS/logs/recorder.log"

while true; do
  STATE="$(curl -s --max-time 1 http://127.0.0.1:8000/get_state \
    | sed -n 's/.*"state": *"\([A-Z]*\)".*/\1/p')"
  case "$STATE" in
    AUTONOMOUS|MANUAL)
      if [ -z "$REC_PID" ] || ! kill -0 "$REC_PID" 2>/dev/null; then
        [ -n "$REC_PID" ] && echo "[$(date -u +%FT%TZ)] recorder died, restarting" >> "$WS/logs/recorder.log"
        start_run "$STATE"
      fi
      ;;
    IDLE|STOPPED)
      stop_run
      ;;
  esac
  sleep 0.25
done
