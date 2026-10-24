#!/usr/bin/env bash
# Isolated experimental GPS soft prior with full LiDAR-only search fallback.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BAG="${1:?Specify localization, localization_from2, localization_3}"
case "$BAG" in localization|localization_from2|localization_3) ;; *) exit 2;; esac
GPS_MODE="${GPS_MODE:-on}"
case "$GPS_MODE" in on|off) ;; *) exit 2;; esac
RATE="${RATE:-0.75}"
POSE_LIMIT="${POSE_LIMIT:-false}"
[[ "$POSE_LIMIT" == true || "$POSE_LIMIT" == false ]] || exit 2
MAP="${MAP:-campus_full_lio_226s_20261019}"
SUFFIX=""; [[ "$POSE_LIMIT" == true ]] && SUFFIX="_pose_limited"
NAME="full_${BAG}_mh_20261019"
OUT="$ROOT/outputs/multihyp_lidar_only_20261019/$NAME"
[[ ! -e "$OUT/status.txt" ]] || { echo "Existing run $OUT" >&2;exit 2; }
mkdir -p "$OUT"
START="$(python3 - "$ROOT" "$BAG" <<'PY'
import json,sys
print(json.load(open(sys.argv[1]+'/outputs/looooo_integrity_20261019/'+sys.argv[2]+'.json'))['sensor_timestamps']['lidar']['first_stamp'])
PY
)"
docker run --rm --name "mh_full_${BAG}" --memory=7g --memory-swap=8g \
  -v "$ROOT/../rosbag/looooo:/bags:ro" -v "$ROOT/maps/$MAP:/map:ro" \
  -v "$ROOT/outputs/lidar_global_holdout_20261019:/odom:ro" \
  -v "$ROOT/outputs/multihyp_lidar_only_20261019:/results" \
  -v "$ROOT/config/calibration/zedx_current.yaml:/calibration.yaml:ro" \
  -v "$ROOT/scripts/monitor_multihyp_lidar.py:/monitor.py:ro" \
  -e "BAG=$BAG" -e "NAME=$NAME" -e "RATE=$RATE" -e "START=$START" -e "GPS_MODE=$GPS_MODE" -e "POSE_LIMIT=$POSE_LIMIT" \
  --entrypoint /bin/bash fast_livo2:mh_lidar_diag_20261019 -lc '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /ws/devel/setup.bash
    out="/results/$NAME";export ROS_HOME="$out/ros_home";mkdir -p "$ROS_HOME"
    roscore > "$out/roscore.log" 2>&1 & core=$!
    cleanup(){ kill -INT "${node:-}" "${mon:-}" 2>/dev/null || true;sleep 2;kill "$core" 2>/dev/null || true; }
    trap cleanup EXIT INT TERM
    sleep 3;rosparam set /use_sim_time true;rosparam load /calibration.yaml
    enabled=false; [[ "$GPS_MODE" == on ]] && enabled=true
    rosrun fast_livo2 step3_global_acquirer _map_db:=/map \
     _lidar_topic:=/lidar103/velodyne_points _fast_odom_topic:=/aft_mapped_to_init \
     _query_hz:=1.0 _source_voxel:=0.35 _descriptor_candidates:=12 _geometric_candidates:=3 \
     _gicp_max_fitness:=0.15 _required_consistent:=2 _tracking_hz:=1.0 \
     _tracking_candidates:=3 _tracking_submap_cache_slots:=12 \
     _tracking_gicp_max_iterations:=16 _tracking_update_anchor:=true \
     _mh_enable:=true _mh_required_hits:=3 _mh_max_reanchor_m:=4.0 \
     _gps_soft_prior_enable:=false _gps_preference_radius_m:=25.0 \
     _gps_max_age_sec:=0.65 _gps_max_covariance_m2:=100.0 \
     _tracking_publish_relative_motion_limit_enable:="$POSE_LIMIT" \
     _tracking_publish_relative_correction_speed_mps:=1.5 \
     _tracking_publish_relative_yaw_rate_deg_s:=12.0 \
     >"$out/step3.log" 2>&1 & node=$!
    sleep 5; kill -0 "$node" || { tail -40 "$out/step3.log";exit 3; }
    python3 /monitor.py --out "$out/events.json" --bag-start "$START" >"$out/monitor.log" 2>&1 & mon=$!
    sleep 2;echo "playback_started $(date -Is) mh=on gps=off" >"$out/status.txt"
    set +e
    timeout --signal=INT 700 rosbag play --quiet --clock -r "$RATE" \
      "/bags/$BAG.bag" "/odom/${BAG}_lidar_only_odom.bag" \
      --topics /lidar103/velodyne_points /aft_mapped_to_init \
      >"$out/rosbag_play.log" 2>&1
    rc=$?;set -e
    echo "playback_exit=$rc $(date -Is)" >>"$out/status.txt"
    sleep 3;cleanup;trap - EXIT INT TERM
    echo "run_complete $(date -Is)" >>"$out/status.txt"
    exit "$rc"
  '
