#!/usr/bin/env bash
# Isolated exhaustive single-sensor failures + independent September holdout.
# No deployment/promotion; all heavy ROS replays are serialized by the runner.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2
DEV="$ROOT/results/camera_lidar_imu_gps/HEALTHY_LOOP_MOTION_BRIDGE_DEV_20261015"
HOLD="$ROOT/results/18_sep_2026_combined/HEALTHY_LOOP_MOTION_BRIDGE_HOLDOUT_20261015"
A="$ROOT/step3_global/healthy_validation_assets_20261014"
ORIGINAL="$ROOT/launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch"
REF="$A/loopclosed_reference_recovered_equivalent.csv"
HREF="$ROOT/results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv"
export STEP3_IMAGE=fast_livo2_failsafe:healthy-loop-motion-bridge-dev-20261015
export STEP3_LAUNCH_FILE="$ORIGINAL"
export STEP3_CONFIG="$ROOT/config/failsafe_rosbag.yaml"
export STEP3_CALIBRATION_FILE="$ROOT/config/calibration/recorded_cart_bag.yaml"
export STEP3_VISUAL_MAP="$A/final_camera_rgb_lidar_voxel_15cm.pcd"
export STEP3_VISUAL_MAP_IMAGES="$A/visual_map_images"
export STEP3_VISUAL_MAP_TRAJ="$REF"
export STEP3_ORB_CACHE="$A/orb_cache_dev_20261015/visual_orb_retrieval_exact.npz"
export STEP3_VISUAL_PLACE_ENABLE=true
export STEP3_LIDAR_FEATURE_PNP=false STEP3_LIDAR_AKAZE=false
export STEP3_TRACKING_ESIKF_MEASUREMENT_ENABLE=false
export STEP3_GYRO_BIAS_FEEDBACK_ENABLE=false
export STEP3_ANCHORED_LOCAL_FEEDBACK_ENABLE=false
export STEP3_DEFERRED_SCAN_ENABLE=true
export STEP3_CONTINUITY_RETRY_ENABLE=true
export PLAY_RATE=1
mkdir -p "$DEV" "$HOLD"
STATUS="$DEV/four_modes_and_holdout_status.txt"
printf 'BEGIN_ALL_VALIDATIONS %s\n' "$(date -Iseconds)" > "$STATUS"

case_run(){
  local case="$1" sensor="$2" base="$3"
  echo "BEGIN $case $(date -Iseconds)" | tee -a "$STATUS"
  if ! bash "$ROOT/scripts/run_step3_global_case.sh" "$case" "$sensor" > "$base/${case}.execution.log" 2>&1; then
    echo "FAILED $case rc=runner $(date -Iseconds)" | tee -a "$STATUS"
    tail -18 "$base/${case}.execution.log"
    return 1
  fi
  if [[ ! -s "$base/$case/metrics_exact.txt" || ! -s "$base/$case/plots/07_orange_trajectory_on_pcd_clean.png" ]]; then
    echo "FAILED $case output_missing $(date -Iseconds)" | tee -a "$STATUS"
    return 1
  fi
  grep -E '^reference_disagreement_3d_rmse_m=|^reference_disagreement_failure_rmse3d_m=|^step_max_m=|^n=|^duration=' \
    "$base/$case/metrics_exact.txt" | tee -a "$STATUS"
  echo "COMPLETE $case $(date -Iseconds)" | tee -a "$STATUS"
}

export BAG_PATH=/home/devil/Desktop/car/alive-repo/rosbag/camera_lidar_imu_gps.bag
export STEP3_OUT_BASE="$DEV" STEP3_REFERENCE="$REF" PLAY_DURATION=290
export DROP_START=125 DROP_DURATION=40
for spec in CAMERA_FAIL:camera LIDAR_FAIL:lidar IMU_FAIL:imu; do
  mode="${spec%%:*}"; sensor="${spec#*:}"
  case_run "${mode}_BRIDGE_290S" "$sensor" "$DEV" || exit 3
done

echo "GENERATING_DEV_COMPARISON $(date -Iseconds)" | tee -a "$STATUS"
if ! python3 "$ROOT/scripts/report_development_bag_four_modes.py" --help >/dev/null 2>&1; then
  echo 'ERROR: cannot invoke four-mode development comparison' | tee -a "$STATUS"; exit 4
fi
if ! python3 "$ROOT/scripts/report_development_bag_four_modes.py" \
      --reference "$REF" --map-pcd "$ROOT/step3_global/map_db_sweeps_fast/cache/global_map.pcd" \
      --out "$DEV/COMPARISON_BRIDGE" \
      --healthy "$DEV/HEALTHY_BRIDGE_290S" \
      --camera "$DEV/CAMERA_FAIL_BRIDGE_290S" \
      --lidar "$DEV/LIDAR_FAIL_BRIDGE_290S" \
      --imu "$DEV/IMU_FAIL_BRIDGE_290S" > "$DEV/COMPARISON_BRIDGE.log" 2>&1; then
    echo 'FAILED DEV_COMPARISON' | tee -a "$STATUS"
    tail -20 "$DEV/COMPARISON_BRIDGE.log"
    exit 4
fi
echo "DEV_FOUR_MODES_COMPLETE $(date -Iseconds)" | tee -a "$STATUS"

# Freeze exactly the same executable, config and geometry for the holdout.
# The September reference is solely an evaluation proxy; the matcher doesn't
# ingest it, and no map is regenerated from this holdout.
export BAG_PATH=/home/devil/Desktop/car/alive-repo/rosbag/18_sep_2026_combined.bag
export STEP3_OUT_BASE="$HOLD" STEP3_REFERENCE="$HREF" PLAY_DURATION=220
export DROP_START=90 DROP_DURATION=40
for spec in HEALTHY:none CAMERA_FAIL:camera LIDAR_FAIL:lidar IMU_FAIL:imu; do
  mode="${spec%%:*}"; sensor="${spec#*:}"
  case_run "${mode}_HOLDOUT_220S" "$sensor" "$HOLD" || exit 5
done
if ! python3 "$ROOT/scripts/summarize_heldout_turn_aware_20261014.py" \
    --results "$HOLD" --reference "$HREF" \
    --map "$ROOT/step3_global/map_db_sweeps_fast/cache/global_map.pcd" \
    --out "$HOLD/COMPARISON" > "$HOLD/COMPARISON.log" 2>&1; then
  echo 'FAILED HOLDOUT_COMPARISON' | tee -a "$STATUS"
  tail -20 "$HOLD/COMPARISON.log"; exit 6
fi
echo "ALL_FOUR_MODES_AND_HOLDOUT_FINISHED $(date -Iseconds)" | tee -a "$STATUS"
