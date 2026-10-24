#!/usr/bin/env bash
# Read-only bag repro: 20-55s LiDAR-loss continuity seeded to saved healthy
# FAST-LIVO2 odom available ONLY before t=20s. No global authority, no /pose.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:-localization_3}"
case "$BAG" in localization|localization_from2|localization_3);; *)exit 2;;esac
RUN="${RUN:-${BAG}_seeded_20to55_offline_ros1}"
CAMERA_BLACKOUT="${CAMERA_BLACKOUT:-false}"
case "$CAMERA_BLACKOUT" in true|false);; *)exit 2;;esac
OUT="$ROOT/outputs/zedx_rgbd_imu_fallback_20261020/$RUN"
[[ ! -e "$OUT/status.txt" && ! -e "$OUT/roscore.log" ]] || { echo 'Existing run' >&2;exit 4; }
mkdir -p "$OUT"
FIRST="$(python3 - "$ROOT" "$BAG" <<'PY'
import json,sys
print(json.load(open(sys.argv[1]+'/outputs/looooo_integrity_20261019/'+sys.argv[2]+'.json'))['sensor_timestamps']['lidar']['first_stamp'])
PY
)"
docker run --rm --memory=7g --memory-swap=8g --name "zedx_seeded_${RUN}" \
 -v "$ROOT:/repo:ro" -v "$ROOT/../rosbag/looooo:/bags:ro" \
 -v "$ROOT/outputs/lidar_global_holdout_20261019:/odom:ro" \
 -v "$ROOT/outputs/zedx_rgbd_imu_fallback_20261020:/results" \
 -e RUN="$RUN" -e BAG="$BAG" -e FIRST="$FIRST" -e CAMERA_BLACKOUT="$CAMERA_BLACKOUT" \
 --entrypoint /bin/bash fast_livo2:dev -lc '
 set -eo pipefail; source /opt/ros/noetic/setup.bash;source /ws/devel/setup.bash
 out="/results/$RUN";export ROS_HOME="$out/ros_home";mkdir -p "$ROS_HOME"
 roscore >"$out/roscore.log" 2>&1 & core=$!
 vo=;bridge=;fixture=;monitor=;cand_monitor=;camera_relay=
 cleanup(){ for pid in "$vo" "$bridge" "$fixture" "$monitor" "$cand_monitor" "$camera_relay";do if [[ -n "$pid" ]];then kill -INT "$pid" 2>/dev/null || true;fi;done;sleep 2;kill -INT "$core" 2>/dev/null || true; }
 trap cleanup EXIT INT TERM
 sleep 3;rosparam set /use_sim_time true;rosparam load /repo/config/calibration/zedx_current.yaml
 rosparam set /fresh_zedx_camera_imu_odom/offline_only true
 rosparam set /fresh_zedx_camera_imu_odom/frame_period_sec 0.065
 rosparam set /fresh_zedx_camera_imu_odom/max_output_gap_sec 0.65
 rosparam set /fresh_local_odom_fallback_bridge/offline_only true
 rosparam set /fresh_local_odom_fallback_bridge/max_seed_age_sec 0.6
 rosparam set /fresh_local_odom_fallback_bridge/max_output_gap_sec 0.65
 rosparam set /offline_only_lidar_odom_outage_fixture/bag_first_lidar_stamp "$FIRST"
 rosparam set /offline_only_lidar_odom_outage_fixture/outage_start_sec 20.0
 rosparam set /offline_only_lidar_odom_outage_fixture/outage_end_sec 55.0
 if [[ "$CAMERA_BLACKOUT" == true ]];then
   rosparam set /offline_camera_dropout_relay/first_lidar_stamp "$FIRST"
   rosparam set /offline_camera_dropout_relay/drop_start_sec 30.0
   rosparam set /offline_camera_dropout_relay/drop_end_sec 35.0
   rosparam set /fresh_zedx_camera_imu_odom/image_topic /fallback_test/rgb_relay
   rosparam set /fresh_zedx_camera_imu_odom/depth_topic /fallback_test/depth_relay
   python3 /repo/scripts/relay_camera_dropout_for_fallback.py >"$out/camera_relay.log" 2>&1 & camera_relay=$!
 fi
 python3 /repo/scripts/fresh_zedx_camera_imu_odom.py >"$out/vo_node.log" 2>&1 & vo=$!
 python3 /repo/scripts/fresh_local_odom_fallback_bridge.py >"$out/bridge.log" 2>&1 & bridge=$!
 python3 /repo/scripts/simulate_lidar_odom_outage_rosbag.py >"$out/fixture.log" 2>&1 & fixture=$!
 python3 /repo/scripts/monitor_seeded_camera_imu_fallback.py --out "$out/seeded_events.json" >"$out/seeded_monitor.log" 2>&1 & monitor=$!
 python3 /repo/scripts/monitor_zedx_camera_imu_odom.py --output "$out/vo_events.json" >"$out/vo_monitor.log" 2>&1 & cand_monitor=$!
 sleep 3;for pid in "$vo" "$bridge" "$fixture" "$monitor" "$cand_monitor" "$camera_relay";do kill -0 "$pid" || { echo "A node exited unexpectedly";tail -30 "$out/bridge.log";exit 5; };done
 echo "START $(date -Ins)" >"$out/status.txt"
 set +e
 timeout --signal=INT 230 rosbag play --quiet --clock -r 0.75 \
    "/bags/$BAG.bag" "/odom/${BAG}_lidar_only_odom.bag" \
    --topics /zedx/zed_node/left/image_rect_color \
     /zedx/zed_node/depth/depth_registered /zedx/zed_node/imu/data /aft_mapped_to_init \
     >"$out/play.log" 2>&1
 rc=$?;set -e
 echo "PLAY_EXIT=$rc $(date -Ins)" >>"$out/status.txt"
 sleep 3;cleanup;trap - EXIT INT TERM
 echo "COMPLETE $(date -Ins)" >>"$out/status.txt"
 exit "$rc"
 '
