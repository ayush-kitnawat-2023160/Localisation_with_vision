#!/usr/bin/env bash
# Isolated ROS1 OFFLINE test: real Step3 LiDAR hypotheses + XFeat visual
# candidates + read-only cross-sensor observer, deliberately NEVER /pose.
# Requires pre-running localhost XFeat learned_float_flann worker at port 18091.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:-localization_3}"
case "$BAG" in localization|localization_from2|localization_3) ;; *)echo "Unexpected bag $BAG" >&2;exit 2;;esac
RATE="${RATE:-0.75}"
MH_REACQUIRE_HZ="${MH_REACQUIRE_HZ:-0.33}"
IMAGE="${IMAGE:-fast_livo2:mh_precise_20261020}"
MH_VISUAL_SEED="${MH_VISUAL_SEED:-false}"
FAULT_LIDAR="${FAULT_LIDAR:-false}"
HANDOFF_REVIEW="${HANDOFF_REVIEW:-false}"
case "$FAULT_LIDAR" in true|false) ;; *) exit 2;; esac
case "$HANDOFF_REVIEW" in true|false) ;; *) exit 2;; esac
case "$MH_VISUAL_SEED" in true|false) ;; *) exit 2;; esac
RUN="${RUN:-ros1_live_mh_xfeat_${BAG}_20261020}"
OUT="$ROOT/outputs/learned_visual_20261020/$RUN"
[[ ! -f "$OUT/replay.status" ]] || { echo "Existing run $OUT" >&2;exit 2; }
[[ ! -d "$OUT/ros_home" ]] || { echo "Existing in-progress directory $OUT" >&2;exit 2; }
curl -fsS http://127.0.0.1:18091/health | python3 -c 'import json,sys;x=json.load(sys.stdin);assert x["ready"] and not x["gps_used"] and x["backend"]=="xfeat_mnn" and x["retrieval_mode"]=="learned_float_flann"' || { echo "Dedicated localhost XFeat worker not ready" >&2;exit 4; }
if ss -lnt '( sport = :11311 )' | grep -q 11311;then echo 'ROS master already listening on host port 11311; refusing to interfere' >&2;exit 5;fi
FIRST="$(python3 - "$ROOT" "$BAG" <<'PY'
import sys,json
x=json.load(open(sys.argv[1]+'/outputs/looooo_integrity_20261019/'+sys.argv[2]+'.json'))
print(x['sensor_timestamps']['lidar']['first_stamp'])
PY
)"
mkdir -p "$OUT"
docker run --rm --network host --name "visual_lidar_mh_live_${BAG}" --memory=8g --memory-swap=10g \
 -v "$ROOT:/repo:ro" \
 -v "$ROOT/../rosbag/looooo:/bags:ro" \
 -v "$ROOT/maps/campus_full_lio_226s_20261019:/map:ro" \
 -v "$ROOT/outputs/lidar_global_holdout_20261019:/odom:ro" \
 -v "$ROOT/outputs/learned_visual_20261020:/results" \
 -v "$ROOT/config/calibration/zedx_current.yaml:/calibration.yaml:ro" \
 -e BAG="$BAG" -e RATE="$RATE" -e RUN="$RUN" -e FIRST="$FIRST" -e MH_REACQUIRE_HZ="$MH_REACQUIRE_HZ" -e MH_VISUAL_SEED="$MH_VISUAL_SEED" -e FAULT_LIDAR="$FAULT_LIDAR" -e HANDOFF_REVIEW="$HANDOFF_REVIEW" \
 --entrypoint /bin/bash "$IMAGE" -lc '
   set -eo pipefail
   source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash
   export ROS_MASTER_URI=http://127.0.0.1:11311
   export ROS_IP=127.0.0.1
   out="/results/$RUN"; export ROS_HOME="$out/ros_home";mkdir -p "$ROS_HOME"
   ros_master=;node=;bridge=;observer=;monitor=;mh_monitor=;fault_relay=;handoff_reviewer=
   cleanup(){
     for p in "$node" "$bridge" "$observer" "$monitor" "$mh_monitor" "$fault_relay" "$handoff_reviewer"; do
       if [[ -n "$p" ]];then kill -INT "$p" 2>/dev/null || true;fi
     done
     sleep 3
     if [[ -n "$ros_master" ]];then kill -INT "$ros_master" 2>/dev/null || true;fi
   }
   trap cleanup EXIT INT TERM
   roscore > "$out/roscore.log" 2>&1 & ros_master=$!
   sleep 3
   rosparam set /use_sim_time true
   rosparam load /calibration.yaml
   rosparam set /fresh_visual_global_learned_bridge/always_evaluate_offline true
   rosparam set /fresh_visual_global_learned_bridge/worker_url http://127.0.0.1:18091/localize
   rosparam set /fresh_visual_global_learned_bridge/query_hz 1.0
   rosparam set /fresh_visual_global_learned_bridge/confirm_frames 3
   rosparam set /fresh_visual_global_learned_bridge/request_timeout_sec 1.5
   rosparam set /fresh_multimodal_candidate_observer/max_bank_age_sec 2.5
   rosparam set /fresh_multimodal_candidate_observer/max_bank_to_visual_sensor_offset_sec 1.5
   lidar_topic=/lidar103/velodyne_points;odom_topic=/aft_mapped_to_init
   if [[ "$FAULT_LIDAR" == true ]];then
     lidar_topic=/visual_test/lidar_faulted;odom_topic=/visual_test/odom_faulted
     rosparam set /lidar_odom_topic_failure_relay/lidar_first_stamp "$FIRST"
     rosparam set /lidar_odom_topic_failure_relay/start_sec 26.0
     rosparam set /lidar_odom_topic_failure_relay/end_sec 39.0
     rosparam set /lidar_odom_topic_failure_relay/drop_odom_too true
     python3 /repo/scripts/inject_lidar_odom_replay_faults.py > "$out/fault_relay.log" 2>&1 & fault_relay=$!
   fi
   rosrun fast_livo2 step3_global_acquirer _map_db:=/map \
     _lidar_topic:="$lidar_topic" _fast_odom_topic:="$odom_topic" \
     _query_hz:=1.0 _source_voxel:=0.35 _descriptor_candidates:=12 _geometric_candidates:=3 \
     _gicp_max_fitness:=0.15 _required_consistent:=2 _tracking_hz:=1.0 \
     _tracking_candidates:=3 _tracking_submap_cache_slots:=12 \
     _tracking_gicp_max_iterations:=16 _tracking_update_anchor:=true \
     _mh_enable:=true _mh_required_hits:=3 _mh_max_reanchor_m:=4.0 \
     _tracking_global_reacquire_hz:="$MH_REACQUIRE_HZ" \
     _mh_visual_seed_enable:="$MH_VISUAL_SEED" _mh_visual_seed_max_age_sec:=1.4 \
     _gps_soft_prior_enable:=false > "$out/step3.log" 2>&1 & node=$!
   python3 /repo/scripts/fresh_visual_global_learned_bridge.py > "$out/bridge.log" 2>&1 & bridge=$!
   python3 /repo/scripts/fresh_multimodal_candidate_observer.py > "$out/observer.log" 2>&1 & observer=$!
   if [[ "$HANDOFF_REVIEW" == true ]];then
     PYTHONPATH=/repo/scripts:${PYTHONPATH:-} python3 /repo/scripts/fresh_safe_handoff_review.py > "$out/handoff_reviewer.log" 2>&1 & handoff_reviewer=$!
   fi
   python3 /repo/scripts/monitor_learned_visual_ros1.py --output "$out/visual_events.json" > "$out/monitor.log" 2>&1 & monitor=$!
   python3 /repo/scripts/monitor_multihyp_lidar.py --out "$out/lidar_events.json" --bag-start "$FIRST" > "$out/lidar_monitor.log" 2>&1 & mh_monitor=$!
   sleep 7
   for p in "$node" "$bridge" "$observer" "$monitor" "$mh_monitor" "$fault_relay" "$handoff_reviewer";do [[ -z "$p" ]] && continue;kill -0 "$p" || { echo "One of the ROS nodes died during init";tail -25 "$out/bridge.log";tail -25 "$out/step3.log";exit 9;};done
   rosparam get /step3_global_acquirer/gps_soft_prior_enable > "$out/gps_enabled.txt" 2>&1 || true
   echo "started $(date -Ins)" > "$out/replay.status"
   set +e
   timeout --signal=INT 430 rosbag play --quiet --clock -r "$RATE" \
      "/bags/$BAG.bag" "/odom/${BAG}_lidar_only_odom.bag" \
      --topics /zedx/zed_node/left/image_rect_color /lidar103/velodyne_points /aft_mapped_to_init \
      > "$out/rosbag_play.log" 2>&1
   rc=$?;set -e
   echo "bag_exit=$rc $(date -Ins)" >> "$out/replay.status"
   sleep 3; cleanup; trap - EXIT INT TERM
   echo "complete $(date -Ins)" >> "$out/replay.status"
   exit "$rc"
 '
