#!/usr/bin/env bash
# Evaluate true *cold* map initialization from a different physical scan in
# localization_3.bag, without reusing its earlier global lock/anchor.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
START_SEC="${1:?e.g. 10,22,36,45,54}"
DURATION_SEC="${2:-14}"
GPS_MODE="${GPS_MODE:-off}"
RATE="${RATE:-0.75}"
case "$START_SEC" in 10|22|36|45|50|52|54) ;; *) echo 'Allowed starts 10,22,36,45,50,52,54' >&2;exit 2;;esac
case "$GPS_MODE" in on|off);; *)exit 2;;esac
NAME="cold_${START_SEC}s_${DURATION_SEC}s_mh"
OUT="$ROOT/outputs/multihyp_lidar_only_20261019/$NAME"
[[ ! -e "$OUT/status.txt" ]] || { echo "Refusing to overwrite $OUT" >&2;exit 3; }
mkdir -p "$OUT"
FIRST_LIDAR="$(python3 - "$ROOT" <<'PY'
import json,sys
p=sys.argv[1]+'/outputs/looooo_integrity_20261019/localization_3.json'
print(json.load(open(p))['sensor_timestamps']['lidar']['first_stamp'])
PY
)"
exec docker run --rm --name "cold_mh_local3_${START_SEC}" --memory=7g --memory-swap=8g \
 -v "$ROOT/../rosbag/looooo:/bags:ro" \
 -v "$ROOT/maps/campus_full_lio_226s_20261019:/map:ro" \
 -v "$ROOT/outputs/lidar_global_holdout_20261019:/odom:ro" \
 -v "$ROOT/outputs/multihyp_lidar_only_20261019:/results" \
 -v "$ROOT/config/calibration/zedx_current.yaml:/calibration.yaml:ro" \
 -v "$ROOT/scripts/monitor_multihyp_lidar.py:/monitor.py:ro" \
 --env "START_SEC=$START_SEC" --env "DURATION_SEC=$DURATION_SEC" --env "FIRST_LIDAR=$FIRST_LIDAR" \
 --env "GPS_MODE=$GPS_MODE" --env "NAME=$NAME" --env "RATE=$RATE" \
 --entrypoint /bin/bash fast_livo2:mh_lidar_diag_20261019 -lc '
  set -eo pipefail
  source /opt/ros/noetic/setup.bash;source /ws/devel/setup.bash
  out="/results/$NAME";export ROS_HOME="$out/ros_home";mkdir -p "$ROS_HOME"
  roscore >"$out/roscore.log" 2>&1 & core=$!
  cleanup(){ kill -INT "${node:-}" "${monitor:-}" 2>/dev/null || true;sleep 1;kill "$core" 2>/dev/null || true; }
  trap cleanup EXIT INT TERM
  sleep 2
  rosparam set /use_sim_time true;rosparam load /calibration.yaml
  enabled=false; [[ "$GPS_MODE" == on ]] && enabled=true
  rosrun fast_livo2 step3_global_acquirer _map_db:=/map \
    _lidar_topic:=/lidar103/velodyne_points _fast_odom_topic:=/aft_mapped_to_init \
    _query_hz:=1.0 _source_voxel:=0.35 _descriptor_candidates:=12 \
    _geometric_candidates:=3 _gicp_max_fitness:=0.15 _required_consistent:=2 \
    _tracking_hz:=1.0 _tracking_candidates:=3 _tracking_submap_cache_slots:=12 \
    _tracking_gicp_max_iterations:=16 _tracking_update_anchor:=true \
    _mh_enable:=true _mh_required_hits:=3 _mh_max_reanchor_m:=4.0 \
    _gps_soft_prior_enable:=false _gps_preference_radius_m:=25.0 \
    _gps_max_age_sec:=0.65 _gps_max_covariance_m2:=100.0 \
    >"$out/step3.log" 2>&1 & node=$!
  sleep 4;kill -0 "$node" || { tail -50 "$out/step3.log";exit 4;}
  python3 /monitor.py --out "$out/events.json" --bag-start "$FIRST_LIDAR" >"$out/monitor.log" 2>&1 & monitor=$!
  sleep 2
  printf "started %s start=%ss duration=%ss gps=%s\n" "$(date -Is)" "$START_SEC" "$DURATION_SEC" "$GPS_MODE" >"$out/status.txt"
  set +e
  timeout --signal=INT 200 rosbag play --quiet --clock -r "$RATE" \
    --start "$START_SEC" --duration "$DURATION_SEC" \
    /bags/localization_3.bag /odom/localization_3_lidar_only_odom.bag \
    --topics /lidar103/velodyne_points /aft_mapped_to_init \
    >"$out/rosbag_play.log" 2>&1
  rc=$?;set -e
  echo "playback_exit=$rc $(date -Is)" >>"$out/status.txt"
  sleep 2;cleanup;trap - EXIT INT TERM
  echo "run_complete $(date -Is)" >>"$out/status.txt"
  exit "$rc"
 '
