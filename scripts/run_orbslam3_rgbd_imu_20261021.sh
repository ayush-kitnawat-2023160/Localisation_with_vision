#!/usr/bin/env bash
# Independent offline ROS1 ORB-SLAM3 RGB-D+IMU candidate-only benchmark.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BAG="${BAG:-localization_3}"
RATE="${RATE:-1.0}"
MODE="${MODE:-rgbd_imu}"
DURATION="${DURATION:-20}"
RUN="${RUN:-orb_rgbd_imu_20261021}"
case "$BAG" in localization|localization_from2|localization_3) ;; *) exit 2;;esac
case "$MODE" in rgbd|rgbd_imu) ;; *) exit 2;;esac
[[ "$DURATION" =~ ^[0-9]+$ ]] || exit 2
OUT="$ROOT/outputs/orbslam3_fallback_benchmark_20261021/$RUN"
[[ ! -e "$OUT" ]] || { echo "Refusing overwrite $OUT" >&2;exit 2; }
test -x "$ROOT/outputs/orbslam3_fallback_benchmark_20261021/ws/devel/lib/orb_slam3_offline/rgbd_imu_candidate"
mkdir -p "$OUT"
docker run --rm --name "orb_rgbd_offline_$BAG" --ipc host --shm-size 2g \
 -v "$ROOT/outputs/orbslam3_fallback_benchmark_20261021/ws:/benchmark_ws:ro" \
 -v "$ROOT/outputs/orbslam3_fallback_benchmark_20261021/config:/config:ro" \
 -v "$ROOT/../rosbag/looooo:/bags:ro" \
 -v "$ROOT/scripts/monitor_orbslam3_rgbd_imu_20261021.py:/monitor.py:ro" \
 -v "$ROOT/scripts/profile_orbslam3_proc_20261021.py:/profile.py:ro" \
 -v "$OUT:/results" \
 -e BAG="$BAG" -e RATE="$RATE" -e DURATION="$DURATION" -e MODE="$MODE" \
 --entrypoint bash orb-slam3-alive:official -lc '
 set -eo pipefail
 source /opt/ros/noetic/setup.bash
 source /benchmark_ws/devel/setup.bash
 ros_master=;slam=;mon=
 cleanup(){
   for pid in "$slam" "$mon";do [[ -z "$pid" ]] || kill -INT "$pid" 2>/dev/null || true;done
   sleep 6
   [[ -z "$ros_master" ]] || kill -INT "$ros_master" 2>/dev/null || true
 }
 trap cleanup EXIT INT TERM
 roscore >/results/roscore.log 2>&1 & ros_master=$!
 sleep 3
 rosparam set /use_sim_time true
 python3 /profile.py --output /results/process_cpu_ram.json -- \
   /benchmark_ws/devel/lib/orb_slam3_offline/rgbd_imu_candidate \
   /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt \
   /config/zedx_rgbd_imu_provisional.yaml /results/frame_compute.csv "$MODE" \
     > /results/orbslam3.log 2>&1 & slam=$!
 python3 /monitor.py --output /results/pose_events.json --mode "$MODE" \
     > /results/monitor.log 2>&1 & mon=$!
 echo "SLAM_NODE_START_EPOCH=$(date +%s.%N)" > /results/status.txt
 sleep 15
 kill -0 "$slam" || { echo "SLAM_EARLY_EXIT" >>/results/status.txt;tail -60 /results/orbslam3.log;exit 4; }
 kill -0 "$mon" || { echo "MONITOR_EARLY_EXIT" >>/results/status.txt;exit 4; }
 echo "PLAY_START_EPOCH=$(date +%s.%N)" >> /results/status.txt
 set +e
 timeout --signal=INT 200 rosbag play --quiet --clock -r "$RATE" \
    --duration "$DURATION" /bags/$BAG.bag --topics \
    /zedx/zed_node/left/image_rect_color \
    /zedx/zed_node/depth/depth_registered \
    /zedx/zed_node/imu/data > /results/play.log 2>&1
 playrc=$?
 set -e
 echo "PLAY_EXIT=$playrc" >>/results/status.txt
 echo "PLAY_END_EPOCH=$(date +%s.%N)" >>/results/status.txt
 sleep 5
 cleanup
 sleep 3
 trap - EXIT INT TERM
 echo "COMPLETE=$(date -Ins)" >>/results/status.txt
 exit "$playrc"
 '
