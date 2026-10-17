#!/usr/bin/env bash
# Isolation: tests only localisation code and new opt-in recovery-map limits.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/results/LOCALIZATION_ONLY_COMPLETION_20261017/recovery_voxel_budget"
mkdir -p "$OUT"
cd "$ROOT"
export STEP3_IMAGE='fast_livo2_failsafe:recovery-voxel-budget-localization-dev-20261017'
export STEP3_LAUNCH_FILE="$ROOT/launch/fast_livo2_recovery_voxel_budget_dev_20261017.launch"
export STEP3_CONFIG="$ROOT/config/failsafe_recovery_voxel_budget_dev_20261017.yaml"
export STEP3_CALIBRATION_FILE="$ROOT/config/calibration/recorded_cart_bag.yaml"
export STEP3_MAP_LOCALIZER_CONFIG="$ROOT/config/map_failsafe_recorded_cart.yaml"
export STEP3_PRIOR_MAP="$ROOT/step3_global/map_db_sweeps_fast/cache/global_map.pcd"
export STEP3_VISUAL_MAP="$ROOT/step3_global/healthy_validation_assets_20261014/final_camera_rgb_lidar_voxel_15cm.pcd"
export STEP3_VISUAL_MAP_IMAGES="$ROOT/step3_global/healthy_validation_assets_20261014/visual_map_images"
export STEP3_VISUAL_MAP_TRAJ="$ROOT/step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv"
export STEP3_ORB_CACHE="$ROOT/step3_global/healthy_validation_assets_20261014/orb_cache_dev_20261015/visual_orb_retrieval_exact.npz"
export STEP3_VISUAL_PLACE_ENABLE=true STEP3_LIDAR_FEATURE_PNP=false STEP3_LIDAR_AKAZE=false
export STEP3_TRACKING_ESIKF_MEASUREMENT_ENABLE=false STEP3_GYRO_BIAS_FEEDBACK_ENABLE=false
export STEP3_ANCHORED_LOCAL_FEEDBACK_ENABLE=false STEP3_DEFERRED_SCAN_ENABLE=true
export STEP3_CONTINUITY_RETRY_ENABLE=true
export PLAY_RATE=1 DROP_DURATION=40
printf 'image=%s\n' "$(docker image inspect "$STEP3_IMAGE" --format '{{.Id}}')" > "$OUT/source_image.txt"
for bag in sept18 main;do
 case "$bag" in
  sept18)
   export BAG_PATH="/home/devil/Desktop/car/alive-repo/rosbag/18_sep_2026_combined.bag"
   export STEP3_REFERENCE="$ROOT/results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv"
   export STEP3_OUT_BASE="$OUT/sept18" PLAY_DURATION=165 DROP_START=90
   ;;
  main)
   export BAG_PATH="/home/devil/Desktop/car/alive-repo/rosbag/camera_lidar_imu_gps.bag"
   export STEP3_REFERENCE="$ROOT/step3_global/healthy_validation_assets_20261014/loopclosed_reference_recovered_equivalent.csv"
   export STEP3_OUT_BASE="$OUT/main" PLAY_DURATION=190 DROP_START=125
   ;;
 esac
 case_name="IMU_RECOVERY_MAP_BUDGET_$bag"_"1X"
 printf 'START bag=%s case=%s at=%s\n' "$bag" "$case_name" "$(date -Iseconds)" | tee -a "$OUT/run_status.log"
 if ! bash scripts/run_step3_global_case.sh "$case_name" imu > "$OUT/$bag.execution.log" 2>&1;then
  printf 'FAILED bag=%s at=%s\n' "$bag" "$(date -Iseconds)" | tee -a "$OUT/run_status.log"
  tail -50 "$OUT/$bag.execution.log"
  exit 2
 fi
 printf 'COMPLETE bag=%s at=%s\n' "$bag" "$(date -Iseconds)" | tee -a "$OUT/run_status.log"
 grep -E '^reference_disagreement_3d_rmse_m=|^reference_disagreement_failure_rmse3d_m=|^reference_disagreement_post_failure_20s_rmse3d_m=|^step_max_m=' "$STEP3_OUT_BASE/$case_name/metrics_exact.txt" | tee -a "$OUT/run_status.log"
done
echo 'ALL_TWO_BAG_REPLAYS_FINISHED' | tee -a "$OUT/run_status.log"
