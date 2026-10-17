#!/usr/bin/env bash
# Controlled A/B. Run exactly ONE case per invocation. Never replay two bags
# concurrently: heavy LIO + visual callbacks compete and invalidate metrics.
set -euo pipefail
MODE="${1:?usage: benchmark_healthy_visual_fastpath_20261015.sh <baseline|fastpath> [90|290]}"
DURATION="${2:-90}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
A="$ROOT/step3_global/healthy_validation_assets_20261014"
OUT="$ROOT/results/camera_lidar_imu_gps/HEALTHY_VISUAL_FASTPATH_BENCH_20261015"
mkdir -p "$OUT"
case "$MODE" in
 baseline)
   export STEP3_IMAGE=fast_livo2_failsafe:healthy-gap-handoff-dev-20261014
   export STEP3_CONFIG="$ROOT/config/failsafe_rosbag.yaml";;
 fastpath)
   export STEP3_IMAGE=fast_livo2_failsafe:visual-gated-fastpath-dev-20261014
   export STEP3_CONFIG="$ROOT/config/failsafe_rosbag_visual_fastpath_dev.yaml";;
 *) echo 'bad mode' >&2; exit 2;;
esac
export STEP3_OUT_BASE="$OUT" STEP3_LAUNCH_FILE="$ROOT/launch/fast_livo2_step3_global_healthy_gap_handoff_dev.launch"
export STEP3_CALIBRATION_FILE="$ROOT/config/calibration/recorded_cart_bag.yaml"
export STEP3_REFERENCE="$A/loopclosed_reference_recovered_equivalent.csv"
export STEP3_VISUAL_MAP="$A/final_camera_rgb_lidar_voxel_15cm.pcd"
export STEP3_VISUAL_MAP_IMAGES="$A/visual_map_images" STEP3_VISUAL_MAP_TRAJ="$STEP3_REFERENCE"
export STEP3_VISUAL_PLACE_ENABLE=true STEP3_LIDAR_FEATURE_PNP=false STEP3_LIDAR_AKAZE=false
export STEP3_TRACKING_ESIKF_MEASUREMENT_ENABLE=false STEP3_GYRO_BIAS_FEEDBACK_ENABLE=false
export STEP3_ANCHORED_LOCAL_FEEDBACK_ENABLE=false STEP3_DEFERRED_SCAN_ENABLE=true
export STEP3_CONTINUITY_RETRY_ENABLE=true PLAY_DURATION="$DURATION" PLAY_RATE=1 DROP_START=125 DROP_DURATION=40
CASE="HEALTHY_${MODE^^}_${DURATION}S"
echo "BEGIN $(date -Iseconds) mode=$MODE duration=$DURATION" | tee -a "$OUT/validation_status.txt"
bash "$ROOT/scripts/run_step3_global_case.sh" "$CASE" none > "$OUT/$CASE.execution.log" 2>&1 &
pid=$!
echo 'wall_time,cpu_percent,mem_usage,mem_percent,block_io' > "$OUT/$CASE.resource_samples.csv"
while kill -0 "$pid" 2>/dev/null; do
  ts="$(date -Iseconds)"
  row="$(docker stats --no-stream --format '{{.CPUPerc}},{{.MemUsage}},{{.MemPerc}},{{.BlockIO}}' "fastlivo_step3_$CASE" 2>/dev/null | head -1 || : )"
  [[ -z "$row" ]] || echo "$ts,$row" >> "$OUT/$CASE.resource_samples.csv"
  sleep 5
done
wait "$pid" || { echo "FAILED $CASE" | tee -a "$OUT/validation_status.txt"; tail -20 "$OUT/$CASE.execution.log"; exit 3; }
echo "COMPLETE $CASE $(date -Iseconds)" | tee -a "$OUT/validation_status.txt"
