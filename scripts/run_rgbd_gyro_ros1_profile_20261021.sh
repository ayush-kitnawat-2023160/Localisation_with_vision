#!/usr/bin/env bash
# Isolated ROS1 RGB-D + gyro candidate-only benchmark with process CPU/RSS.
# Compare only estimator processes; this does not profile GPU stereo depth SDK.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${BAG:-localization_3}"
RATE="${RATE:-1.0}"
RUN="${RUN:-rgbd_gyro_${BAG}_profile_20261021}"
case "$BAG" in localization|localization_from2|localization_3) ;; *) exit 2;;esac
OUT="$ROOT/outputs/rgbd_fallback_benchmark_20261021/$RUN"
[[ ! -e "$OUT" ]] || { echo "Refusing to overwrite $OUT" >&2;exit 3; }
mkdir -p "$OUT"
docker run --rm --name "rgbd_gyro_profile_${BAG}" \
 -v "$ROOT:/repo:ro" -v "$ROOT/../rosbag/looooo:/bags:ro" \
 -v "$OUT:/results" -e BAG="$BAG" -e RATE="$RATE" \
 --entrypoint /bin/bash fast_livo2:dev -lc '
   set -eo pipefail
   source /opt/ros/noetic/setup.bash
   source /ws/devel/setup.bash
   core=;est=;monitor=
   cleanup(){
     for p in "$est" "$monitor";do [[ -z "$p" ]] || kill -INT "$p" 2>/dev/null || true;done
     sleep 3
     [[ -z "$core" ]] || kill -INT "$core" 2>/dev/null || true
   }
   trap cleanup EXIT INT TERM
   roscore > /results/roscore.log 2>&1 & core=$!
   sleep 3
   rosparam set /use_sim_time true
   rosparam load /repo/config/calibration/zedx_current.yaml
   rosparam set /fresh_zedx_camera_imu_odom/offline_only true
   rosparam set /fresh_zedx_camera_imu_odom/require_imu_for_healthy true
   rosparam set /fresh_zedx_camera_imu_odom/frame_period_sec 0.065
   rosparam set /fresh_zedx_camera_imu_odom/image_scale 0.65
   rosparam set /fresh_zedx_camera_imu_odom/max_output_gap_sec 0.65
   rosparam set /fresh_zedx_camera_imu_odom/max_image_silence_sec 0.7
   python3 /repo/scripts/profile_vio_process_20261021.py \
      --out /results/rgbd_process_resources.json -- \
      python3 /repo/scripts/fresh_zedx_camera_imu_odom.py \
      > /results/rgbd_node.log 2>&1 & est=$!
   python3 /repo/scripts/monitor_zedx_camera_imu_odom.py \
      --output /results/pose_events.json > /results/monitor.log 2>&1 & monitor=$!
   sleep 5
   kill -0 "$est" && kill -0 "$monitor" || { echo "Node exited before bag playback";exit 5; }
   echo "PLAY_STARTED $(date -Ins)" > /results/status.txt
   set +e
   timeout --signal=INT 210 rosbag play --quiet --clock -r "$RATE" \
      "/bags/$BAG.bag" --topics \
      /zedx/zed_node/left/image_rect_color \
      /zedx/zed_node/depth/depth_registered \
      /zedx/zed_node/imu/data > /results/rosbag_play.log 2>&1
   rc=$?
   set -e
   echo "PLAY_EXIT=$rc $(date -Ins)" >> /results/status.txt
   sleep 5
   cleanup
   sleep 2
   trap - EXIT INT TERM
   echo "COMPLETE $(date -Ins)" >> /results/status.txt
   exit "$rc"
 '
