#!/usr/bin/env bash
# Held-out LiDAR ScanContext/GICP test with separately precomputed FAST-LIVO2
# odometry. It never publishes the vehicle's /pose or uses GPS as a search prior.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUN="$1"
case "$RUN" in localization|localization_from2|localization_3) ;; *) echo "Invalid bag" >&2; exit 2;; esac
RATE="$(echo "$(printenv RATE || echo 0.75)")"
RATE_TAG="$(echo "$RATE" | tr -d '.')"
NAME="global_${RUN}_rate${RATE_TAG}_20261019"
OUT="$ROOT/outputs/lidar_global_holdout_20261019/$NAME"
if [[ -e "$OUT/status.txt" ]];then echo "Refusing overwrite $OUT" >&2; exit 3; fi
mkdir -p "$OUT"
BAG_START="$(python3 - "$RUN" "$ROOT" <<'PY'
import json,sys
d=json.load(open(sys.argv[2]+'/outputs/looooo_integrity_20261019/'+sys.argv[1]+'.json'))
print(d['sensor_timestamps']['lidar']['first_stamp'])
PY
)"
CONTAINER="global_holdout_${RUN}_${RATE_TAG}"
echo "$RUN bag_start=$BAG_START rate=$RATE output=$OUT"
docker run --rm --name "$CONTAINER" --memory=7g --memory-swap=8g \
  -v "$ROOT/../rosbag/looooo:/bags:ro" \
  -v "$ROOT/maps/campus_full_lio_226s_20261019:/map:ro" \
  -v "$ROOT/outputs/lidar_global_holdout_20261019:/results" \
  -v "$ROOT/config/calibration/zedx_current.yaml:/calibration.yaml:ro" \
  -v "$ROOT/scripts/monitor_lidar_global_lock.py:/monitor.py:ro" \
  --env "BAG=$RUN" --env "BAG_START=$BAG_START" --env "RATE=$RATE" --env "NAME=$NAME" \
  --entrypoint /bin/bash fast_livo2:dev -lc '
    set -euo pipefail
    set +u
    source /opt/ros/noetic/setup.bash
    source /ws/devel/setup.bash
    set -u
    out="/results/$NAME"
    export ROS_HOME="$out/ros_home"
    mkdir -p "$ROS_HOME"
    roscore >"$out/roscore.log" 2>&1 & core=$!
    node=""; monitor=""
    cleanup(){
      if [[ -n "$node" ]]; then kill -INT "$node" 2>/dev/null || true; fi
      if [[ -n "$monitor" ]]; then kill -INT "$monitor" 2>/dev/null || true; fi
      sleep 1
      kill "$core" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM
    sleep 3
    rosparam set /use_sim_time true
    rosparam load /calibration.yaml
    rosrun fast_livo2 step3_global_acquirer \
       _map_db:=/map \
       _lidar_topic:=/lidar103/velodyne_points \
       _fast_odom_topic:=/aft_mapped_to_init \
       _query_hz:=1.0 _source_voxel:=0.35 \
       _descriptor_candidates:=12 _geometric_candidates:=3 \
       _gicp_max_fitness:=0.15 _required_consistent:=2 \
       _tracking_hz:=1.0 _tracking_candidates:=3 \
       _tracking_submap_cache_slots:=12 \
       _tracking_gicp_max_iterations:=16 \
       _tracking_update_anchor:=true >"$out/step3.log" 2>&1 & node=$!
    sleep 5
    if ! kill -0 "$node" 2>/dev/null;then echo "Step3 crashed:";tail -45 "$out/step3.log";exit 4;fi
    python3 /monitor.py --out "$out/events.json" --bag-start "$BAG_START" >"$out/monitor.log" 2>&1 & monitor=$!
    sleep 2
    echo "bagplay_start $(date -Is)" >"$out/status.txt"
    set +e
    timeout --signal=INT 700 rosbag play --quiet --clock -r "$RATE" \
      "/bags/$BAG.bag" "/results/${BAG}_lidar_only_odom.bag" \
      --topics /lidar103/velodyne_points /aft_mapped_to_init /mavros/global_position/raw/fix \
      >"$out/rosbag_play.log" 2>&1
    rc=$?
    set -e
    echo "bagplay_exit=$rc $(date -Is)" >>"$out/status.txt"
    sleep 4
    cleanup
    trap - EXIT INT TERM
    echo "run_complete $(date -Is)" >>"$out/status.txt"
    if [[ "$rc" != 0 ]];then exit "$rc";fi
  '
