#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
: "${INITIAL_X:?Set INITIAL_X to the approximate starting X in the PCD map}"
: "${INITIAL_Y:?Set INITIAL_Y to the approximate starting Y in the PCD map}"
: "${INITIAL_YAW:?Set INITIAL_YAW in radians}"
ROSBAG_DIR="${ROSBAG_DIR:-$PACKAGE_DIR/../../rosbag}"
MAP_DIR="${MAP_DIR:-$PACKAGE_DIR/maps}"
BAG="${1:-/data/rosbag/bags/camera_lidar_imu_gps.bag}"
CHECK_ARGS=("$BAG" "--x=$INITIAL_X" "--y=$INITIAL_Y" "--yaw=$INITIAL_YAW"
  "--duration=${DURATION:-20}" "--rate=${RATE:-0.5}"
  "--search-xy=${SEARCH_XY:-5}" "--search-yaw=${SEARCH_YAW:-20}"
  "--map-path=/data/maps/full_campus.pcd")
if [[ "${FULL:-0}" == 1 ]]; then CHECK_ARGS+=(--full); fi
# Unique, disposable container; source and recorded data remain read-only.
# Rebuild the mounted source so an older cached image cannot mask a code fix.
docker run --rm --network none --env ROS_HOSTNAME=localhost --env ROS_IP=127.0.0.1 \
  --volume "$PACKAGE_DIR:/ws/src/Fast_Livo2:ro" \
  --volume "$(cd "$ROSBAG_DIR" && pwd):/data/rosbag:ro" \
  --volume "$(cd "$MAP_DIR" && pwd):/data/maps:ro" \
  "${FAST_LIVO2_IMAGE:-fast_livo2:dev}" bash -lc '
    set -e
    source /opt/ros/noetic/setup.bash
    cd /ws
    catkin_make -j2 -l2 > /tmp/build.log 2>&1 || { cat /tmp/build.log; exit 1; }
    source /ws/devel/setup.bash
    roscore >/tmp/master.log 2>&1 &
    master_pid=$!
    trap "kill $master_pid 2>/dev/null || true" EXIT
    sleep 2
    python3 /ws/src/Fast_Livo2/tools/check_global_replay.py "$@" || {
      tail -n 100 /tmp/floam-replay/launch.log
      exit 1
    }
  ' bash "${CHECK_ARGS[@]}"
