#!/usr/bin/env bash
# Fresh ROS-bag replay with no saved global map or previous run data.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [[ $# -ge 1 ]]; then
  BAG_REL="$1"
else
  BAG_REL="looooo/localization.bag"
fi
PLAY_RATE=1.0
if [[ $# -ge 2 ]]; then PLAY_RATE="$2"; fi

case "$BAG_REL" in
  /*|*..*|*/*/*/*) echo "Give a bag path RELATIVE to ../rosbag (e.g. looooo/localization.bag)" >&2; exit 2;;
esac
if [[ ! -f "$ROOT/../rosbag/$BAG_REL" ]]; then
  echo "Bag not found: $ROOT/../rosbag/$BAG_REL" >&2
  exit 2
fi
if ! docker image inspect fast_livo2:dev >/dev/null 2>&1; then
  echo "Docker image missing: first run ./docker/build.sh" >&2
  exit 2
fi

echo "Playing $BAG_REL at ${PLAY_RATE}x with saved-map localization DISABLED"
"$ROOT/docker/run.sh" bash -lc '
  set -euo pipefail
  roscore >/tmp/fresh_zedx_roscore.log 2>&1 &
  ROSCORE_PID=$!
  LAUNCH_PID=""
  cleanup() {
    if [[ -n "$LAUNCH_PID" ]]; then kill -INT "$LAUNCH_PID" 2>/dev/null || true; fi
    kill "$ROSCORE_PID" 2>/dev/null || true
  }
  trap cleanup EXIT INT TERM
  sleep 3
  roslaunch fast_livo2 fast_livo2_zedx_fresh_rosbag.launch record_trajectory:=false >/tmp/fresh_zedx_launch.log 2>&1 &
  LAUNCH_PID=$!
  sleep 4
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    cat /tmp/fresh_zedx_launch.log
    exit 2
  fi
  echo "--- launching /data/rosbag/$1 ---"
  rosbag play --clock -r "$2" "/data/rosbag/$1"
  echo "--- launch diagnostics ---"
  tail -n 35 /tmp/fresh_zedx_launch.log
' _ "$BAG_REL" "$PLAY_RATE"
