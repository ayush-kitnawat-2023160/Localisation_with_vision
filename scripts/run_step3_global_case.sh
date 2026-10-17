#!/usr/bin/env bash
set -euo pipefail

CASE="${1:?usage: run_step3_global_case.sh <case_name> <none|camera|lidar|imu>}"
DROP_SENSOR="${2:-none}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_PATH="${BAG_PATH:-/home/devil/Desktop/car/alive-repo/rosbag/camera_lidar_imu_gps.bag}"
MAP_DB="${STEP3_MAP_DB:-$ROOT/step3_global/map_db_sweeps_fast}"
IMAGE="${STEP3_IMAGE:-fast_livo2_failsafe:dev}"
VISUAL_MAP="${STEP3_VISUAL_MAP:-/home/devil/Desktop/car/alive-repo/temp_fast_livo_2/final_failsafe_maps/final_camera_rgb_lidar_voxel_15cm.pcd}"
VISUAL_PLACE_ENABLE="${STEP3_VISUAL_PLACE_ENABLE:-false}"
LIDAR_FEATURE_PNP="${STEP3_LIDAR_FEATURE_PNP:-false}"
LIDAR_AKAZE="${STEP3_LIDAR_AKAZE:-false}"
AKAZE_CACHE="${STEP3_AKAZE_CACHE:-$ROOT/step3_global/visual_akaze_clahe_640.npz}"
VISUAL_MAP_IMAGES="${STEP3_VISUAL_MAP_IMAGES:-/home/devil/Desktop/car/alive-repo/temp_fast_livo_2/output_image3d_map_complete_v3/livo/image}"
VISUAL_MAP_TRAJ="${STEP3_VISUAL_MAP_TRAJ:-/home/devil/Desktop/car/alive-repo/temp_fast_livo_2/output_image_voxel_loopclosed/optimized_trajectory.csv}"
MAP_LOCALIZER_CONFIG="${STEP3_MAP_LOCALIZER_CONFIG:-$ROOT/config/map_failsafe_recorded_cart.yaml}"
STEP3_LAUNCH_FILE="${STEP3_LAUNCH_FILE:-$ROOT/launch/fast_livo2_step3_global_rosbag.launch}"
DROP_START="${DROP_START:-15.0}"
DROP_DURATION="${DROP_DURATION:-20.0}"
PLAY_DURATION="${PLAY_DURATION:-50}"
PLAY_RATE="${PLAY_RATE:-1.0}"
LIO_UPDATE_GUARD="${STEP3_LIO_GUARD_ENABLE:-false}"
TRACKING_ESIKF_MEASUREMENT="${STEP3_TRACKING_ESIKF_MEASUREMENT_ENABLE:-false}"
GYRO_BIAS_FEEDBACK="${STEP3_GYRO_BIAS_FEEDBACK_ENABLE:-false}"
ANCHORED_LOCAL_FEEDBACK="${STEP3_ANCHORED_LOCAL_FEEDBACK_ENABLE:-false}"
DEFERRED_SCANS="${STEP3_DEFERRED_SCAN_ENABLE:-false}"
CONTINUITY_RETRY="${STEP3_CONTINUITY_RETRY_ENABLE:-false}"
if ! awk -v r="$PLAY_RATE" 'BEGIN {exit !(r>0 && r<=2)}'; then
  echo "Invalid PLAY_RATE=$PLAY_RATE; expected 0 < rate <= 2" >&2; exit 2
fi
if [[ "$(realpath -m "$BAG_PATH")" == "/home/devil/Desktop/car/alive-repo/rosbag/18_sep_2026_combined.bag" ]]; then
  DEFAULT_OUT_BASE="$ROOT/results/18_sep_2026_combined"
  DEFAULT_18SEP_REFERENCE="$ROOT/results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv"
else
  DEFAULT_OUT_BASE="$ROOT/results/camera_lidar_imu_gps/failsafe_v2_continuous_map"
  DEFAULT_18SEP_REFERENCE=""
fi
OUT_BASE="${STEP3_OUT_BASE:-$DEFAULT_OUT_BASE}"
OUT="$OUT_BASE/$CASE"
CONFIG_SRC="${STEP3_CONFIG:-$ROOT/config/failsafe_rosbag.yaml}"
CALIBRATION_SRC="${STEP3_CALIBRATION_FILE:-$ROOT/config/calibration/recorded_cart_bag.yaml}"
PRIOR_MAP_SRC="${STEP3_PRIOR_MAP:-$ROOT/step3_global/map_db_sweeps_fast/cache/global_map.pcd}"
PYTHON="${STEP3_PYTHON:-/home/devil/miniconda3/bin/python}"
REFERENCE="${STEP3_REFERENCE:-$DEFAULT_18SEP_REFERENCE}"

case "$DROP_SENSOR" in none|camera|lidar|imu) ;; *) echo "invalid sensor: $DROP_SENSOR" >&2; exit 2;; esac
[[ -f "$BAG_PATH" ]] || { echo "missing bag: $BAG_PATH" >&2; exit 2; }
[[ -d "$MAP_DB" ]] || { echo "missing map db: $MAP_DB" >&2; exit 2; }
[[ -f "$CONFIG_SRC" ]] || { echo "missing config: $CONFIG_SRC" >&2; exit 2; }
[[ -f "$CALIBRATION_SRC" ]] || { echo "missing calibration: $CALIBRATION_SRC" >&2; exit 2; }
[[ -f "$PRIOR_MAP_SRC" ]] || { echo "missing prior map: $PRIOR_MAP_SRC" >&2; exit 2; }
[[ -f "$VISUAL_MAP" ]] || { echo "missing visual map: $VISUAL_MAP" >&2; exit 2; }
if [[ "$VISUAL_PLACE_ENABLE" == "true" ]]; then
  [[ -d "$VISUAL_MAP_IMAGES" ]] || { echo "missing visual images: $VISUAL_MAP_IMAGES" >&2; exit 2; }
  [[ -f "$VISUAL_MAP_TRAJ" ]] || { echo "missing visual trajectory: $VISUAL_MAP_TRAJ" >&2; exit 2; }
fi
[[ -f "$MAP_LOCALIZER_CONFIG" ]] || { echo "missing map localizer config: $MAP_LOCALIZER_CONFIG" >&2; exit 2; }
if [[ "$LIDAR_AKAZE" == "true" ]]; then
  [[ "$LIDAR_FEATURE_PNP" == "true" ]] || { echo "AKAZE mode requires feature PnP mode" >&2; exit 2; }
  [[ -f "$AKAZE_CACHE" ]] || { echo "missing saved-map AKAZE cache: $AKAZE_CACHE" >&2; exit 2; }
fi
[[ -f "$STEP3_LAUNCH_FILE" ]] || { echo "missing Step-3 launch file: $STEP3_LAUNCH_FILE" >&2; exit 2; }
[[ -x "$PYTHON" ]] || { echo "missing plotting/evaluation Python: $PYTHON" >&2; exit 2; }
[[ -n "$REFERENCE" && -f "$REFERENCE" ]] || { echo "Reference missing: set STEP3_REFERENCE to a trajectory from this exact bag. Only the known Sep 18 bag auto-selects its held-out LiDAR/PCD proxy." >&2; exit 2; }
if [[ "$REFERENCE" == "$DEFAULT_18SEP_REFERENCE" ]]; then
  echo "[STEP3 RUN] using Sep 18 held-out LiDAR/map proxy; NOT surveyed ground truth"
fi

# GICP, RGB-D feature tracking, and the estimator compete for the same small
# set of CPU cores and RAM. Two *different* ROS bag validation cases running
# simultaneously can change RGB-D confidence, trigger IMU_ONLY_BRIDGE at a
# different time, and invalidate the comparison. Serialize every case under
# one repository-wide lock as well as preserving the per-case output lock.
# Use file descriptor 8 independently of the per-case descriptor 9.
mkdir -p "$ROOT/results"
exec 8>"$ROOT/results/.step3_global_replay.lock"
if ! flock -n 8; then
  echo "[STEP3 RUN] another case is active in this repository; replay refused without touching output" >&2
  exit 75
fi
# The lock cannot see an already-running job started before this fix, nor a
# replay started outside this script. Explicitly detect their Docker names.
if docker ps --format '{{.Names}}' | grep -Eq '^fastlivo_step3_'; then
  echo "[STEP3 RUN] another Step-3 Docker replay is running; replay refused without touching output" >&2
  exit 75
fi

# A second same-case replay must also never delete an output directory while
# the first Docker process has it bind-mounted. Lock BEFORE touching results.
NAME="fastlivo_step3_${CASE//[^a-zA-Z0-9_.-]/_}"
mkdir -p "$OUT_BASE/.case_locks"
exec 9>"$OUT_BASE/.case_locks/$CASE.lock"
if ! flock -n 9; then
  echo "[STEP3 RUN] same case is already active: $CASE; existing results untouched" >&2
  exit 75
fi
if [[ "$(docker inspect --format '{{.State.Running}}' "$NAME" 2>/dev/null || true)" == true ]]; then
  echo "[STEP3 RUN] Docker container $NAME is still active; results untouched" >&2
  exit 75
fi

rm -rf "$OUT"; mkdir -p "$OUT"
cp "$CONFIG_SRC" "$OUT/config.yaml"
cp "$CALIBRATION_SRC" "$OUT/calibration.yaml"
if [[ "$LIO_UPDATE_GUARD" == "true" ]]; then
  # Keep source config untouched: enable the experimental ESIKF measurement
  # sanity gate only inside this replay's copied config.
  grep -q '^  healthy_visual_pose_fusion_enable:' "$OUT/config.yaml" || {
    echo "cannot enable LIO guard: failsafe YAML anchor missing" >&2; exit 2;
  }
  sed -i '/^  healthy_visual_pose_fusion_enable:/a\  healthy_lidar_update_guard_enable: true\n  healthy_lidar_max_update_translation_m: 0.75\n  healthy_lidar_max_update_rotation_deg: 7.0\n  healthy_lidar_max_update_velocity_mps: 3.0' "$OUT/config.yaml"
fi
cat > "$OUT/run_metadata.txt" <<EOF
case=$CASE
sensor=$DROP_SENSOR
bag_path=$BAG_PATH
reference=$REFERENCE
reference_kind=unverified_trajectory_comparison_not_ground_truth
play_duration_s=$PLAY_DURATION
play_rate=$PLAY_RATE
healthy_lio_update_guard=$LIO_UPDATE_GUARD
tracking_esikf_measurement=$TRACKING_ESIKF_MEASUREMENT
tracking_gyro_bias_feedback=$GYRO_BIAS_FEEDBACK
tracking_anchored_local_feedback=$ANCHORED_LOCAL_FEEDBACK
tracking_deferred_scan=$DEFERRED_SCANS
tracking_continuity_retry=$CONTINUITY_RETRY
calibration_source=$CALIBRATION_SRC
prior_map_source=$PRIOR_MAP_SRC
config_source=$CONFIG_SRC
failure_start_s=$DROP_START
failure_duration_s=$DROP_DURATION
lidar_feature_pnp=$LIDAR_FEATURE_PNP
lidar_akaze=$LIDAR_AKAZE
failure_end_s=$(awk "BEGIN {print $DROP_START+$DROP_DURATION}")
EOF
BAG_DIR="$(dirname "$BAG_PATH")"; BAG_NAME="$(basename "$BAG_PATH")"
# Remove only a stopped leftover. Never force-stop a running replay.
docker rm "$NAME" >/dev/null 2>&1 || true
AKAZE_MOUNT=()
if [[ -f "$AKAZE_CACHE" ]]; then
  AKAZE_MOUNT=(-v "$AKAZE_CACHE:/visual_akaze_cache.npz:ro")
fi
ORB_CACHE_MOUNT=()
ORB_LAUNCH_ARG=""
if [[ -n "${STEP3_ORB_CACHE:-}" ]]; then
  [[ -f "$STEP3_ORB_CACHE" ]] || { echo "missing opt-in ORB cache: $STEP3_ORB_CACHE" >&2; exit 2; }
  grep -q 'name="visual_place_orb_cache_path"' "$STEP3_LAUNCH_FILE" || {
    echo "launch does not support opt-in ORB cache: $STEP3_LAUNCH_FILE" >&2; exit 2;
  }
  ORB_CACHE_MOUNT=(-v "$STEP3_ORB_CACHE:/visual_orb_cache.npz:ro")
  ORB_LAUNCH_ARG="visual_place_orb_cache_path:=/visual_orb_cache.npz"
fi

echo "[STEP3 RUN] case=$CASE drop=$DROP_SENSOR start=$DROP_START duration=$DROP_DURATION"
echo "[STEP3 RUN] map_db=$MAP_DB"
echo "[STEP3 RUN] launch=$STEP3_LAUNCH_FILE"
echo "[STEP3 RUN] out=$OUT"

SENSOR_ADAPTER_SCRIPT="${STEP3_SENSOR_ADAPTER_SCRIPT:-$ROOT/scripts/fast_livo2_sensor_adapter.py}"
[[ -f "$SENSOR_ADAPTER_SCRIPT" ]] || { echo "missing sensor adapter: $SENSOR_ADAPTER_SCRIPT" >&2; exit 2; }

docker run --name "$NAME" --rm --network bridge \
  -v "$BAG_DIR":/bags:ro \
  -v "$OUT/config.yaml":/config.yaml:ro \
  -v "$OUT/calibration.yaml":/calibration.yaml:ro \
  -v "$PRIOR_MAP_SRC":/prior_map.pcd:ro \
  -v "$MAP_DB":/map_db:ro \
  -v "$VISUAL_MAP":/visual_map.pcd:ro \
  -v "$VISUAL_MAP_IMAGES":/visual_map_images:ro \
  -v "$VISUAL_MAP_TRAJ":/visual_map_trajectory.csv:ro \
  "${AKAZE_MOUNT[@]}" \
  "${ORB_CACHE_MOUNT[@]}" \
  -v "$MAP_LOCALIZER_CONFIG":/ws/src/Fast_Livo2/config/map_failsafe_recorded_cart.yaml:ro \
  -v "$OUT":/results \
  -v "$STEP3_LAUNCH_FILE":/ws/src/Fast_Livo2/launch/fast_livo2_step3_global_rosbag.launch:ro \
  -v "$ROOT/launch/fast_livo2_failsafe_rosbag.launch":/ws/src/Fast_Livo2/launch/fast_livo2_failsafe_rosbag.launch:ro \
  -v "$SENSOR_ADAPTER_SCRIPT":/ws/src/Fast_Livo2/scripts/fast_livo2_sensor_adapter.py:ro \
  -v "$ROOT/scripts/rgbd_failsafe_vo.py":/ws/src/Fast_Livo2/scripts/rgbd_failsafe_vo.py:ro \
  -v "$ROOT/scripts/visual_place_hint.py":/ws/src/Fast_Livo2/scripts/visual_place_hint.py:ro \
  "$IMAGE" bash -lc "
set -e; source /opt/ros/noetic/setup.bash; source /ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
taskset -c 0-1 roscore > /results/roscore.log 2>&1 & sleep 2
roslaunch /ws/src/Fast_Livo2/launch/fast_livo2_step3_global_rosbag.launch \\
  config_file:=/config.yaml \\
  calibration_file:=/calibration.yaml \\
  map_db:=/map_db visual_voxel_map:=/visual_map.pcd trajectory_output:=/results/trajectory.csv \\
  visual_place_enable:=$VISUAL_PLACE_ENABLE visual_map_image_dir:=/visual_map_images visual_map_trajectory:=/visual_map_trajectory.csv \\
  $ORB_LAUNCH_ARG \\
  lidar_fail_feature_pnp_enable:=$LIDAR_FEATURE_PNP \\
  lidar_fail_feature_akaze_enable:=$LIDAR_AKAZE \\
  tracking_esikf_measurement_enable:=$TRACKING_ESIKF_MEASUREMENT \\
  tracking_gyro_bias_feedback_enable:=$GYRO_BIAS_FEEDBACK \\
  tracking_anchored_local_feedback_enable:=$ANCHORED_LOCAL_FEEDBACK \\
  tracking_deferred_scan_enable:=$DEFERRED_SCANS \\
  tracking_continuity_retry_enable:=$CONTINUITY_RETRY \\
  drop_sensor:=$DROP_SENSOR drop_start:=$DROP_START drop_duration:=$DROP_DURATION \\
  > /results/launch.log 2>&1 & LAUNCH=\$!
sleep 5
rostopic echo -p /fast_livo2/step3_global_pose > /results/global_pose.csv 2>/results/global_pose.err & E1=\$!
rostopic echo -p /fast_livo2/step3_gicp_map_pose > /results/gicp_map_pose.csv 2>/results/gicp_map_pose.err & E2=\$!
taskset -c 0-1 rosbag play /bags/$BAG_NAME --clock -r $PLAY_RATE --duration $PLAY_DURATION > /results/rosbag.log 2>&1 || true
sleep 2; kill \$E1 \$E2 \$LAUNCH >/dev/null 2>&1 || true; sleep 1
"

echo "[STEP3 RUN] finished $OUT"
echo "[STEP3 RUN] evaluating + plotting"
"$PYTHON" "$ROOT/scripts/evaluate_step3_run.py" "$OUT" --reference "$REFERENCE" --window "$PLAY_DURATION" --sensor "$DROP_SENSOR" --drop-start "$DROP_START" --drop-duration "$DROP_DURATION"
"$PYTHON" "$ROOT/scripts/plot_step3_run.py" "$OUT" --reference "$REFERENCE" --sensor "$DROP_SENSOR" --window "$PLAY_DURATION" --drop-start "$DROP_START" --drop-duration "$DROP_DURATION"
if [[ -f "$ROOT/scripts/plot_step3_pcd_overlay.py" ]]; then
  "$PYTHON" "$ROOT/scripts/plot_step3_pcd_overlay.py" "$OUT" --reference "$REFERENCE" --window "$PLAY_DURATION" --sensor "$DROP_SENSOR" --drop-start "$DROP_START" --drop-duration "$DROP_DURATION" || echo "[STEP3 RUN] warning: PCD overlay plotting failed" >&2
fi
echo "[STEP3 RUN] diagnostics saved under $OUT/plots"
