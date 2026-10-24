#!/usr/bin/env bash
# Complete local estimator + GPS-free LiDAR global + KLT + health integration
# replay. Visual XFeat is conditional on an independently running local worker.
# /pose and any control are always OFF. No prerecorded /aft_mapped_to_init.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${BAG:?Set BAG=localization, BAG=localization_from2, or BAG=localization_3 explicitly}"
RUN="${RUN:-gpsfree_shadow_${BAG}_20261021}"
RATE="${RATE:-0.75}"
IMAGE="${IMAGE:-fast_livo2:mh_recovery_review_20261020}"
VISUAL="${VISUAL:-false}"
DURATION="${DURATION:-45}"
LIDAR_FAULT_START="${LIDAR_FAULT_START:-}"
LIDAR_FAULT_END="${LIDAR_FAULT_END:-}"
case "$BAG" in localization|localization_from2|localization_3) ;; *) echo Invalid_bag >&2;exit 2;;esac
if [[ "$RUN" == *from2* && "$BAG" != localization_from2 ]];then
  echo 'RUN name implies from2 but BAG disagrees; refusing misleading benchmark' >&2;exit 2
fi
if [[ "$RUN" == *local3* && "$BAG" != localization_3 ]];then
  echo 'RUN name implies local3 but BAG disagrees; refusing misleading benchmark' >&2;exit 2
fi
case "$VISUAL" in true|false) ;; *) echo Invalid_visual >&2;exit 2;;esac
[[ "$DURATION" =~ ^[0-9]+$ ]] || exit 2
FAULT=false
if [[ -n "$LIDAR_FAULT_START" || -n "$LIDAR_FAULT_END" ]];then
  [[ "$LIDAR_FAULT_START" =~ ^[0-9]+([.][0-9]+)?$ ]] || exit 2
  [[ "$LIDAR_FAULT_END" =~ ^[0-9]+([.][0-9]+)?$ ]] || exit 2
  python3 - "$LIDAR_FAULT_START" "$LIDAR_FAULT_END" "$DURATION" <<'PY'
import sys
a,b,n=map(float,sys.argv[1:])
assert 0<a<b<n, 'Injected outage must be fully inside explicit bag replay duration'
PY
  FAULT=true
fi
OUT="$ROOT/outputs/deployment_audit_20261021/$RUN"
[[ ! -e "$OUT" ]] || { echo "Refusing to overwrite $OUT";exit 3; }
# This is static; an offline PASS never authorizes vehicle motion.
python3 "$ROOT/scripts/preflight_gpsfree_deployment_20261021.py" \
   --mode offline --bag "$BAG.bag" > /dev/null
# The optional XFeat worker is bound to HOST loopback, therefore a VISUAL=true
# replay needs --network host and must NOT interfere with an existing master.
# VISUAL=false uses Docker's isolated network and its private ROS master;
# another user's host ROS master is not an obstruction.
if [[ "$VISUAL" == true ]] && ss -lnt '( sport = :11311 )' | grep -q 11311;then
  echo 'ROS master already running on host; refusing to disturb it' >&2;exit 4
fi
mkdir -p "$OUT"
printf 'BAG=%s\nRUN=%s\nRATE=%s\nDURATION=%s\nVISUAL=%s\nINJECT_OFFLINE_LIDAR_FAULT=%s\nFAULT_START=%s\nFAULT_END=%s\n' \
   "$BAG" "$RUN" "$RATE" "$DURATION" "$VISUAL" "$FAULT" \
   "$LIDAR_FAULT_START" "$LIDAR_FAULT_END" > "$OUT/experiment_manifest.env"
chmod +x "$ROOT/scripts/deployment_sensor_health_shadow.py" \
         "$ROOT/scripts/fresh_zedx_camera_imu_odom.py" \
         "$ROOT/scripts/fresh_safe_handoff_review.py"
chmod +x "$ROOT/scripts/deployment_klt_continuation_shadow.py"
chmod +x "$ROOT/scripts/relay_lidar_dropout_shadow_20261021.py"
NETWORK_ARGS=()
if [[ "$VISUAL" == true ]]; then
  # Learned visual worker runs on HOST loopback 127.0.0.1:18091.
  # Preflight refuses missing/incorrect/mode-GPS visual workers.
  python3 "$ROOT/scripts/preflight_gpsfree_deployment_20261021.py" \
     --mode offline --bag "$BAG.bag" --require-visual-worker \
     --learned-map "$ROOT/maps/campus_full_visual_xfeat_226s_20261020" > /dev/null
  NETWORK_ARGS=(--network host)
fi
docker run --rm --name "gpsfree_shadow_offline_${BAG}" --memory=10g "${NETWORK_ARGS[@]}" \
  -v "$ROOT:/ws/src/Fast_Livo2:rw" \
  -v "$ROOT/../rosbag/looooo:/bags:ro" \
  -v "$OUT:/result" -e BAG="$BAG" -e RATE="$RATE" \
  -e DURATION="$DURATION" -e VISUAL="$VISUAL" -e FAULT="$FAULT" \
  -e LIDAR_FAULT_START="$LIDAR_FAULT_START" -e LIDAR_FAULT_END="$LIDAR_FAULT_END" \
  --entrypoint bash "$IMAGE" -lc '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /ws/devel/setup.bash
    export ROS_MASTER_URI=http://127.0.0.1:11311
    export ROS_IP=127.0.0.1
    export ROS_HOME=/result/ros_home
    mkdir -p "$ROS_HOME"
    ros_master=;launch=;mon=;relay=
    cleanup(){
      for pid in "$launch" "$mon" "$relay";do [[ -z "$pid" ]] || kill -INT "$pid" 2>/dev/null || true;done
      sleep 5
      [[ -z "$ros_master" ]] || kill -INT "$ros_master" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM
    roscore > /result/roscore.log 2>&1 & ros_master=$!
    sleep 3
    rosparam set /use_sim_time true
    if [[ "$FAULT" == true ]];then
      python3 /ws/src/Fast_Livo2/scripts/relay_lidar_dropout_shadow_20261021.py \
         --start "$LIDAR_FAULT_START" --end "$LIDAR_FAULT_END" \
         > /result/lidar_dropout_relay.log 2>&1 & relay=$!
    fi
    roslaunch /ws/src/Fast_Livo2/launch/deployment_gpsfree_shadow_20261021.launch \
      use_sim_time:=true \
      calibration_file:=/ws/src/Fast_Livo2/config/calibration/zedx_current.yaml \
      map_db:=/ws/src/Fast_Livo2/maps/campus_full_lio_226s_20261019 \
      visual_map_dir:=/ws/src/Fast_Livo2/maps/campus_full_visual_rgbd_226s_20261020/visual \
      lidar_point_time_unit:=seconds \
      learned_visual_enable:="$VISUAL" \
      > /result/roslaunch.log 2>&1 & launch=$!
    python3 /ws/src/Fast_Livo2/scripts/monitor_deployment_shadow_ros1_20261021.py \
      --output /result/monitor.json > /result/monitor.log 2>&1 & mon=$!
    sleep 8
    kill -0 "$launch" && kill -0 "$mon" || { echo 'NODE_FAILED_BEFORE_PLAY' >/result/status.txt;tail -35 /result/roslaunch.log;exit 5; }
    # A required review/health node missing from ROS launch must fail the
    # experiment, never silently pass with most of the graph still running.
    for required in /laserMapping /fast_livo2_sensor_adapter /fresh_lidar_global \
      /fresh_safe_handoff_review /fresh_pose_authority \
      /fresh_zedx_camera_imu_odom /deployment_sensor_health_shadow \
      /deployment_klt_continuation_shadow;do
      rosnode list | grep -Fxq "$required" || {
        echo "MISSING_REQUIRED_NODE=$required" > /result/status.txt
        tail -45 /result/roslaunch.log; exit 7;
      }
    done
    if [[ "$VISUAL" == true ]];then
      rosnode list | grep -Fxq /fresh_learned_visual || {
        echo 'MISSING_LEARNED_VISUAL_NODE' >/result/status.txt;exit 7;
      }
    fi
    if [[ "$FAULT" == true ]];then
      kill -0 "$relay" && rosnode list | grep -Fxq /relay_lidar_dropout_shadow || {
        echo 'FAULT_RELAY_NOT_RUNNING' >/result/status.txt;exit 7;
      }
    fi
    echo "PLAY_STARTED $(date -Ins)" > /result/status.txt
    remap=()
    if [[ "$FAULT" == true ]];then
      remap=(/lidar103/velodyne_points:=/shadow_offline/source_lidar)
    fi
    set +e
    timeout --signal=INT 420 rosbag play --quiet --clock -r "$RATE" \
      --duration "$DURATION" "/bags/$BAG.bag" --topics \
      /lidar103/velodyne_points \
      /mavros/imu/data \
      /zedx/zed_node/left/image_rect_color \
      /zedx/zed_node/depth/depth_registered \
      /zedx/zed_node/imu/data "${remap[@]}" \
      > /result/bag_play.log 2>&1
    play_rc=$?
    set -e
    echo "PLAY_EXIT=$play_rc" >> /result/status.txt
    sleep 5;cleanup;sleep 3;trap - EXIT INT TERM
    echo "FINISHED $(date -Ins)" >> /result/status.txt
    # A rosbag play exit=0 is NOT proof that the estimator nodes worked.
    # The observer saves outputs after graceful shutdown; reject silent
    # ROS-node crashes, missing estimator messages or forbidden publications.
    python3 /ws/src/Fast_Livo2/scripts/validate_deployment_replay_20261021.py \
      --monitor /result/monitor.json --visual "$VISUAL" --fault "$FAULT" \
      --output /result/runtime_postconditions.json
    exit "$play_rc"
  '
