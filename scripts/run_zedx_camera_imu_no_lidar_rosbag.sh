#!/usr/bin/env bash
# Isolated ROS Noetic rosbag playback of ONLY ZED X RGB/depth/IMU.
# No LiDAR messages, FAST-LIVO2 pose, GPS, IMU MAVROS, /pose or cart control.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:-localization_3}"
case "$BAG" in localization|localization_from2|localization_3);; *)echo 'Unknown bag' >&2;exit 2;;esac
RATE="${RATE:-0.75}"
NO_IMU="${NO_IMU:-false}"
PLAY_DURATION="${PLAY_DURATION:-}"
if [[ -n "$PLAY_DURATION" ]];then [[ "$PLAY_DURATION" =~ ^[0-9]+$ ]] || exit 2;fi
case "$NO_IMU" in true|false);; *)exit 2;;esac
RUN="${RUN:-${BAG}_no_lidar_ros1}"
OUT="$ROOT/outputs/zedx_rgbd_imu_fallback_20261020/$RUN"
[[ ! -e "$OUT/status.txt" && ! -e "$OUT/roscore.log" ]] || { echo "Refusing overwrite $OUT" >&2;exit 4;}
mkdir -p "$OUT"
# A dedicated isolated ROS master inside Docker protects any existing ROS1
# master or unrelated containers running on the college computer.
docker run --rm --name "zedx_camera_imu_rosbag_${RUN}" --memory=7g --memory-swap=8g \
 -v "$ROOT:/repo:ro" \
 -v "$ROOT/../rosbag/looooo:/bags:ro" \
 -v "$ROOT/outputs/zedx_rgbd_imu_fallback_20261020:/results" \
 -e BAG="$BAG" -e RUN="$RUN" -e RATE="$RATE" -e NO_IMU="$NO_IMU" -e PLAY_DURATION="$PLAY_DURATION" -e FRAME_PERIOD="${FRAME_PERIOD:-0.065}" --entrypoint /bin/bash fast_livo2:dev -lc '
 set -eo pipefail;source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash
 out="/results/$RUN";export ROS_HOME="$out/ros_home";mkdir -p "$ROS_HOME"
 roscore > "$out/roscore.log" 2>&1 & master=$!
 worker=;monitor=
 cleanup(){ [[ -z "$worker" ]] || kill -INT "$worker" 2>/dev/null || true;[[ -z "$monitor" ]] || kill -INT "$monitor" 2>/dev/null || true; sleep 2;kill -INT "$master" 2>/dev/null || true; }
 trap cleanup EXIT INT TERM
 sleep 3;rosparam set /use_sim_time true;rosparam load /repo/config/calibration/zedx_current.yaml
 rosparam set /fresh_zedx_camera_imu_odom/offline_only true
 rosparam set /fresh_zedx_camera_imu_odom/require_imu_for_healthy true
 rosparam set /fresh_zedx_camera_imu_odom/frame_period_sec "${FRAME_PERIOD:-0.065}"
 rosparam set /fresh_zedx_camera_imu_odom/max_output_gap_sec 0.65
 rosparam set /fresh_zedx_camera_imu_odom/max_image_silence_sec 0.7
 rosparam set /fresh_zedx_camera_imu_odom/image_scale 0.65
 python3 /repo/scripts/fresh_zedx_camera_imu_odom.py > "$out/vo_node.log" 2>&1 & worker=$!
 python3 /repo/scripts/monitor_zedx_camera_imu_odom.py --output "$out/events.json" > "$out/monitor.log" 2>&1 & monitor=$!
 sleep 3
 kill -0 "$worker" && kill -0 "$monitor" || { tail -25 "$out/vo_node.log";exit 5; }
 echo "START $(date -Ins)" > "$out/status.txt"
 set +e
 imu_topic=/zedx/zed_node/imu/data
 [[ "$NO_IMU" == false ]] || imu_topic=/no_real_imu_messages
 duration_arg=()
 if [[ -n "$PLAY_DURATION" ]];then duration_arg=(--duration "$PLAY_DURATION");fi
 timeout --signal=INT 230 rosbag play --quiet --clock -r "$RATE" "${duration_arg[@]}" \
   "/bags/${BAG}.bag" \
   --topics /zedx/zed_node/left/image_rect_color \
    /zedx/zed_node/depth/depth_registered "$imu_topic" \
    > "$out/rosbag_play.log" 2>&1
 rc=$?;set -e
 echo "PLAY_EXIT=$rc $(date -Ins)" >> "$out/status.txt"
 sleep 3;cleanup; trap - EXIT INT TERM
 echo "COMPLETE $(date -Ins)" >> "$out/status.txt"
 exit "$rc"
 '
