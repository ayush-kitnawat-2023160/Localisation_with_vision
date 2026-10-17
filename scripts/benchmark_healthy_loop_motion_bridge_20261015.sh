#!/usr/bin/env bash
# Replays an isolated Step-3 gate change, with identical frozen base and
# precomputed visual map cache for baseline and candidate.
set -euo pipefail
MODE="${1:?usage: benchmark_healthy_loop_motion_bridge_20261015.sh <baseline|bridge> [90|290]}"
DURATION="${2:-90}"
[[ "$DURATION" == 90 || "$DURATION" == 290 ]] || exit 2
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
A="$ROOT/step3_global/healthy_validation_assets_20261014"
OUT="${STEP3_OUT_BASE:-$ROOT/results/camera_lidar_imu_gps/HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015}"
case "$MODE" in
  baseline) export STEP3_IMAGE=fast_livo2_failsafe:healthy-gap-handoff-dev-20261014;;
  bridge) export STEP3_IMAGE=fast_livo2_failsafe:healthy-loop-motion-bridge-dev-20261015;;
  *) exit 2;;
esac
export STEP3_OUT_BASE="$OUT"
export STEP3_LAUNCH_FILE="${STEP3_LAUNCH_FILE:-$ROOT/launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch}"
export STEP3_CONFIG="$ROOT/config/failsafe_rosbag.yaml"
export STEP3_CALIBRATION_FILE="$ROOT/config/calibration/recorded_cart_bag.yaml"
export STEP3_REFERENCE="$A/loopclosed_reference_recovered_equivalent.csv"
export STEP3_VISUAL_MAP="$A/final_camera_rgb_lidar_voxel_15cm.pcd"
export STEP3_VISUAL_MAP_IMAGES="$A/visual_map_images" STEP3_VISUAL_MAP_TRAJ="$STEP3_REFERENCE"
export STEP3_ORB_CACHE="$A/orb_cache_dev_20261015/visual_orb_retrieval_exact.npz"
export STEP3_VISUAL_PLACE_ENABLE=true STEP3_LIDAR_FEATURE_PNP=false STEP3_LIDAR_AKAZE=false
export STEP3_TRACKING_ESIKF_MEASUREMENT_ENABLE=false STEP3_GYRO_BIAS_FEEDBACK_ENABLE=false
export STEP3_ANCHORED_LOCAL_FEEDBACK_ENABLE=false STEP3_DEFERRED_SCAN_ENABLE=true
export STEP3_CONTINUITY_RETRY_ENABLE=true PLAY_DURATION="$DURATION" PLAY_RATE=1
export DROP_START=125 DROP_DURATION=40
CASE="${STEP3_CASE_LABEL:-HEALTHY_${MODE^^}_${DURATION}S}"
mkdir -p "$OUT"
echo "BEGIN $CASE $(date -Iseconds) image=$STEP3_IMAGE" | tee -a "$OUT/validation_status.txt"
bash "$ROOT/scripts/run_step3_global_case.sh" "$CASE" none > "$OUT/$CASE.execution.log" 2>&1 & pid=$!
echo 'wall_time,cpu_percent,mem_usage,mem_percent,block_io' > "$OUT/$CASE.resource_samples.csv"
while kill -0 "$pid" 2>/dev/null; do
  row="$(docker stats --no-stream --format '{{.CPUPerc}},{{.MemUsage}},{{.MemPerc}},{{.BlockIO}}' "fastlivo_step3_$CASE" 2>/dev/null | head -1 || true)"
  [[ -z "$row" ]] || echo "$(date -Iseconds),$row" >> "$OUT/$CASE.resource_samples.csv"
  sleep 5
done
if ! wait "$pid"; then
  echo "FAILED $CASE $(date -Iseconds)" | tee -a "$OUT/validation_status.txt"
  tail -30 "$OUT/$CASE.execution.log"
  exit 3
fi
echo "COMPLETE $CASE $(date -Iseconds)" | tee -a "$OUT/validation_status.txt"
