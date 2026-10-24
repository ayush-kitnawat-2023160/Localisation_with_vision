#!/usr/bin/env bash
# One-command offline RViz demo: image-only visual global candidates, RGB-D +
# ZED-gyro odometry, camera image, and both saved maps. No LiDAR input is played.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:-localization_from2}"
DURATION="${2:-65}"
RATE="${3:-0.75}"
ROS_CONTAINER="${FAILSAFE_RVIZ_CONTAINER:-failsafe_rviz_demo}"
ROS_IMAGE="${FAILSAFE_RVIZ_IMAGE:-fast_livo2:mh_recovery_review_20261020}"
WORKER_IMAGE="${FAILSAFE_XFEAT_IMAGE:-fast_livo2:learned_visual_cpu_20261020}"
WORKER_NAME="failsafe_visual_worker_$$"
WORKER_PORT="18092"
WORKER_URL="http://127.0.0.1:${WORKER_PORT}/localize"
STARTED_WORKER=false

case "$BAG" in localization|localization_from2|localization_3) ;; *)
  echo "BAG must be localization, localization_from2, or localization_3" >&2
  exit 2
  ;;
esac
[[ "$DURATION" =~ ^[0-9]+$ ]] || { echo 'DURATION must be integer seconds' >&2; exit 2; }
[[ "$RATE" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo 'RATE must be a positive number' >&2; exit 2; }
python3 - "$RATE" <<'PY'
import sys
if float(sys.argv[1]) <= 0:
    raise SystemExit('RATE must be positive')
PY

for path in \
  "$ROOT/maps/campus_full_lio_226s_20261019/global_map.pcd" \
  "$ROOT/maps/campus_full_visual_rgbd_226s_20261020/visual/landmarks_preview.pcd" \
  "$ROOT/maps/campus_full_visual_xfeat_226s_20261020/visual/keyframes.csv" \
  "$ROOT/outputs/learned_visual_20261020/vendor/accelerated_features/weights/xfeat.pt" \
  "$ROOT/config/calibration/zedx_current.yaml" \
  "$ROOT/../rosbag/looooo/${BAG}.bag"; do
  [[ -f "$path" ]] || { echo "Missing demo input: $path" >&2; exit 3; }
done

docker image inspect "$ROS_IMAGE" >/dev/null 2>&1 || {
  echo "Missing Docker image: $ROS_IMAGE" >&2
  exit 3
}
docker image inspect "$WORKER_IMAGE" >/dev/null 2>&1 || {
  echo "Missing Docker image: $WORKER_IMAGE" >&2
  exit 3
}

EXPECTED_MAP_SHA="$(sha256sum "$ROOT/maps/campus_full_visual_xfeat_226s_20261020/visual/keyframes.csv" | cut -d ' ' -f 1)"
worker_is_ready() {
  local response
  response="$(curl -fsS --max-time 2 http://127.0.0.1:18091/health 2>/dev/null)" || return 1
  python3 -c 'import json,sys; d=json.loads(sys.stdin.read()); sys.exit(0 if d.get("ready") and d.get("gps_used") is False and d.get("backend")=="xfeat_mnn" and d.get("map_sha256")==sys.argv[1] else 1)' \
    "$EXPECTED_MAP_SHA" <<< "$response"
}

cleanup() {
  if [[ "$STARTED_WORKER" == true ]]; then
    docker stop "$WORKER_NAME" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

if worker_is_ready; then
  WORKER_URL='http://127.0.0.1:18091/localize'
  echo 'Reusing the ready GPS-free XFeat worker on port 18091.'
else
  if command -v ss >/dev/null 2>&1 && ss -lnt '( sport = :18092 )' | grep -q ':18092'; then
    echo 'Port 18092 is occupied and no matching worker was found on port 18091.' >&2
    exit 4
  fi
  docker run -d --rm --name "$WORKER_NAME" --network host --memory=6g \
    -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=2 \
    -v "$ROOT:/repo:ro" --entrypoint bash "$WORKER_IMAGE" -lc \
    'cd /repo && exec python3 -u /repo/scripts/serve_learned_visual_global.py \
      --map /repo/maps/campus_full_visual_xfeat_226s_20261020 \
      --orb-map /repo/maps/campus_full_visual_rgbd_226s_20261020 \
      --vendor /repo/outputs/learned_visual_20261020/vendor \
      --calibration /repo/config/calibration/zedx_current.yaml \
      --backend xfeat_mnn --device cpu --port 18092 \
      --retrieval-mode learned_float_flann' >/dev/null
  STARTED_WORKER=true
  ready=false
  for _ in $(seq 1 90); do
    response="$(curl -fsS --max-time 2 "http://127.0.0.1:${WORKER_PORT}/health" 2>/dev/null || true)"
    if [[ -n "$response" ]] && python3 -c \
      'import json,sys; d=json.loads(sys.stdin.read()); sys.exit(0 if d.get("ready") and d.get("gps_used") is False and d.get("backend")=="xfeat_mnn" and d.get("map_sha256")==sys.argv[1] else 1)' \
      "$EXPECTED_MAP_SHA" <<< "$response"; then
      ready=true
      break
    fi
    sleep 1
  done
  if [[ "$ready" != true ]]; then
    docker logs "$WORKER_NAME" 2>&1 | tail -40 >&2 || true
    echo 'XFeat worker did not become ready with the expected visual map.' >&2
    exit 5
  fi
  WORKER_URL="http://127.0.0.1:${WORKER_PORT}/localize"
  echo "Started GPS-free XFeat worker at ${WORKER_URL}."
fi

if ! docker inspect "$ROS_CONTAINER" >/dev/null 2>&1; then
  XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"
  [[ -r "$XAUTH_FILE" ]] || {
    echo "Cannot read X authority file: $XAUTH_FILE" >&2
    exit 6
  }
  docker run -d --name "$ROS_CONTAINER" --network host --memory=8g \
    -e DISPLAY="${DISPLAY:-:0}" -e XAUTHORITY=/tmp/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$XAUTH_FILE:/tmp/.Xauthority:ro" \
    -v "$ROOT:/ws/src/Fast_Livo2:ro" \
    -v "$ROOT/../rosbag/looooo:/bags:ro" \
    --entrypoint bash "$ROS_IMAGE" -lc \
    'source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash && \
     export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 && roscore' >/dev/null
  echo "Started ROS 1 container $ROS_CONTAINER."
else
  running="$(docker inspect -f '{{.State.Running}}' "$ROS_CONTAINER")"
  if [[ "$running" != true ]]; then
    docker start "$ROS_CONTAINER" >/dev/null
  fi
  echo "Reusing ROS 1 container $ROS_CONTAINER."
fi

echo "Launching RViz demo on $BAG (${DURATION}s bag time at ${RATE}x)."
docker exec -i "$ROS_CONTAINER" bash -s -- "$BAG" "$DURATION" "$RATE" "$WORKER_URL" <<'INNER'
set -eo pipefail
BAG="$1"
DURATION="$2"
RATE="$3"
WORKER_URL="$4"
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash
set -u
export ROS_HOME=/tmp/failsafe_rviz_demo_ros_home
mkdir -p "$ROS_HOME" /tmp/failsafe_rviz_demo_logs

if ! timeout 2 rosparam list >/dev/null 2>&1; then
  echo 'ROS master is not responding in failsafe_rviz_demo.' >&2
  exit 7
fi
rosparam set /use_sim_time true

PIDS=()
RVIZ_PID=''
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  [[ -z "$RVIZ_PID" ]] || kill -INT "$RVIZ_PID" 2>/dev/null || true
  sleep 2
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  [[ -z "$RVIZ_PID" ]] || kill "$RVIZ_PID" 2>/dev/null || true
  for pid in "${PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
  [[ -z "$RVIZ_PID" ]] || wait "$RVIZ_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

start_node() {
  local name="$1"
  shift
  "$@" >"/tmp/failsafe_rviz_demo_logs/${name}.log" 2>&1 &
  PIDS+=("$!")
}

start_node maps roslaunch /ws/src/Fast_Livo2/launch/view_fresh_maps.launch \
  rviz:=false \
  lidar_pcd:=/ws/src/Fast_Livo2/maps/campus_full_lio_226s_20261019/global_map.pcd \
  visual_pcd:=/ws/src/Fast_Livo2/maps/campus_full_visual_rgbd_226s_20261020/visual/landmarks_preview.pcd
start_node visual_global roslaunch /ws/src/Fast_Livo2/launch/fresh_visual_learned_candidate.launch \
  calibration_file:=/ws/src/Fast_Livo2/config/calibration/zedx_current.yaml \
  worker_url:="$WORKER_URL" \
  offline_replay_all_modes:=true observe_lidar_mh:=false
start_node rgbd_gyro_odom roslaunch /ws/src/Fast_Livo2/launch/fresh_zedx_rgbd_imu_candidate_offline.launch \
  use_sim_time:=true \
  calibration_file:=/ws/src/Fast_Livo2/config/calibration/zedx_current.yaml \
  lidar_health_topic:=/failsafe_rviz/no_lidar
start_node paths python3 /ws/src/Fast_Livo2/scripts/rviz_visualization_path_bridge.py

sleep 6
for required in /fresh_lidar_map /fresh_visual_map /fresh_learned_visual \
  /fresh_zedx_camera_imu_odom /rviz_visualization_path_bridge; do
  if ! rosnode list | grep -Fxq "$required"; then
    echo "Required demo node did not start: $required" >&2
    for log in /tmp/failsafe_rviz_demo_logs/*.log; do
      echo "--- $log" >&2
      tail -30 "$log" >&2 || true
    done
    exit 8
  fi
done

rviz -d /ws/src/Fast_Livo2/rviz/failsafe_visual_localization_demo.rviz \
  >/tmp/failsafe_rviz_demo_logs/rviz.log 2>&1 &
RVIZ_PID="$!"
sleep 3
if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
  cat /tmp/failsafe_rviz_demo_logs/rviz.log >&2
  exit 9
fi

PLAY_ARGS=(--quiet --clock -r "$RATE")
if [[ "$DURATION" != 0 ]]; then PLAY_ARGS+=(--duration "$DURATION"); fi
rosbag play "${PLAY_ARGS[@]}" "/bags/${BAG}.bag" --topics \
  /zedx/zed_node/left/image_rect_color \
  /zedx/zed_node/depth/depth_registered \
  /zedx/zed_node/imu/data \
  >/tmp/failsafe_rviz_demo_logs/rosbag_play.log 2>&1 &
PIDS+=("$!")
echo 'RViz is open. Both saved map layers, camera, visual fixes, aligned odometry, and paths are enabled.'
echo 'The RGB-D/gyro path is anchored to the first visual map candidate for display only.'
echo 'Close RViz or press Ctrl-C here to stop the demo. ROS core remains running.'
wait "$RVIZ_PID" || true
INNER
