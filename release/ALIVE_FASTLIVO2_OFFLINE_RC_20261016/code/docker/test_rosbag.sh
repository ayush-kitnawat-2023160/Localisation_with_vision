#!/usr/bin/env bash
set -euo pipefail

BAG="${1:-/data/rosbag/bags/camera_lidar_imu_gps.bag}"
DURATION="${DURATION:-20}"
RATE="${RATE:-0.25}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROSBAG_DIR="${ROSBAG_DIR:-$PACKAGE_DIR/../../rosbag}"
DATA_DIR="$(cd "$ROSBAG_DIR" && pwd)"
MAP_DIR="${MAP_DIR:-$PACKAGE_DIR/maps}"
MAP_DATA_DIR="$(cd "$MAP_DIR" && pwd)"

docker rm -f fast_livo2_dev 2>/dev/null || true
docker run --rm --name fast_livo2_dev \
  --volume "$DATA_DIR:/data/rosbag:ro" \
  --volume "$MAP_DATA_DIR:/data/maps:ro" \
  fast_livo2:dev bash -lc '
    set -e
    source /opt/ros/noetic/setup.bash
    source /ws/devel/setup.bash
    roscore >/tmp/roscore.log 2>&1 &
    MASTER_PID=$!
    trap "kill $MASTER_PID 2>/dev/null || true" EXIT
    sleep 3
    roslaunch fast_livo2 fast_livo2_rosbag.launch rviz:=false >/tmp/fast_livo2.launch.log 2>&1 &
    LAUNCH_PID=$!
    trap "kill $LAUNCH_PID $MASTER_PID 2>/dev/null || true" EXIT
    sleep 8
    timeout 30 rostopic echo -n 1 /fast_livo2/local_odom >/tmp/fast_livo2.odom.log 2>&1 &
    ODOM_WATCH_PID=$!
    timeout 30 rostopic echo -n 1 /fast_livo2/local_pose >/tmp/fast_livo2.pose.log 2>&1 &
    POSE_WATCH_PID=$!
    set +e
    timeout --signal=INT "'"$DURATION"'" rosbag play --clock -r "'"$RATE"'" "'"$BAG"'"
    PLAY_STATUS=$?
    set -e
    sleep 2
    kill "$ODOM_WATCH_PID" "$POSE_WATCH_PID" 2>/dev/null || true
    echo "--- rosnode list ---"
    rosnode list
    echo "--- relevant topic types ---"
    rostopic type /fast_livo2/input/lidar
    rostopic type /fast_livo2/input/imu
    rostopic type /fast_livo2/input/image
    echo "--- estimator output topic types ---"
    rostopic type /aft_mapped_to_init || true
    rostopic type /fast_livo2/local_odom || true
    echo "--- first compatibility odometry ---"
    cat /tmp/fast_livo2.odom.log
    cat /tmp/fast_livo2.pose.log
    echo "--- launch log tail ---"
    tail -n 80 /tmp/fast_livo2.launch.log
    if [[ "$PLAY_STATUS" -ne 0 && "$PLAY_STATUS" -ne 124 && "$PLAY_STATUS" -ne 130 ]]; then
      exit "$PLAY_STATUS"
    fi
  '
