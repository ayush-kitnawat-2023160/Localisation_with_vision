#!/usr/bin/env bash
# Heldout validation only; no parameter tuning and no map/reference regeneration.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)";cd "$ROOT" || exit 2
export BAG_PATH=/home/devil/Desktop/car/alive-repo/rosbag/18_sep_2026_combined.bag
export STEP3_OUT_BASE="$ROOT/results/18_sep_2026_combined/LOOP_TURN_FASTCATCH_HOLDOUT_20261014"
export STEP3_IMAGE=fast_livo2_failsafe:loop-turn-aware-dev-20261014
export STEP3_LAUNCH_FILE="$ROOT/launch/fast_livo2_step3_global_turn_aware_fastcatch_dev.launch"
export STEP3_CONFIG="$ROOT/config/failsafe_rosbag.yaml"
export STEP3_CALIBRATION_FILE="$ROOT/config/calibration/recorded_cart_bag.yaml"
export STEP3_REFERENCE="$ROOT/results/18_sep_2026_combined/HELDOUT_LIDAR_MAP_REFERENCE_20261009/CORNER_FIXED_REFERENCE_20261009/REFERENCE_18SEP_LIDAR_PCD_CORNER_FIXED_20261009.csv"
export STEP3_VISUAL_MAP=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/final_failsafe_maps/final_camera_rgb_lidar_voxel_15cm.pcd
export STEP3_VISUAL_MAP_IMAGES=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/output_image3d_map_complete_v3/livo/image
export STEP3_VISUAL_MAP_TRAJ=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/output_image_voxel_loopclosed/optimized_trajectory.csv
export STEP3_VISUAL_PLACE_ENABLE=true STEP3_LIDAR_FEATURE_PNP=false STEP3_LIDAR_AKAZE=false
export STEP3_TRACKING_ESIKF_MEASUREMENT_ENABLE=false STEP3_GYRO_BIAS_FEEDBACK_ENABLE=false STEP3_ANCHORED_LOCAL_FEEDBACK_ENABLE=false STEP3_CONTINUITY_RETRY_ENABLE=true
export PLAY_DURATION=220 PLAY_RATE=1 DROP_START=90 DROP_DURATION=40
mkdir -p "$STEP3_OUT_BASE"
cat > "$STEP3_OUT_BASE/README.md" <<'EOF'
# Held-out September 18 validation — 2026-10-14

Results are for `18_sep_2026_combined.bag`, 220-second playback at 1x with three 90–130-second sensor failures.
No September reference, saved PCD map, extrinsics, matcher thresholds or model weights were changed after the development candidate was frozen.
Reference is a same-bag LiDAR/PCD geometric proxy with known limitations, NOT independent surveyed ground truth.
The candidate may still have unsafe jumps/drift and must not be deployed merely on full-run RMSE.
EOF
printf 'BEGIN_HELDOUT %s\n' "$(date -Iseconds)" > "$STEP3_OUT_BASE/status.txt"
for case in HEALTHY:none CAMERA_FAIL:camera LIDAR_FAIL:lidar IMU_FAIL:imu;do
 n="${case%%:*}";s="${case#*:}"
 echo "BEGIN $n $(date -Iseconds)"|tee -a "$STEP3_OUT_BASE/status.txt"
 ./scripts/run_step3_global_case.sh "${n}_HOLDOUT_220S" "$s" > "$STEP3_OUT_BASE/${n}_execution.log" 2>&1
 rc=$?
 if [ "$rc" -ne 0 ] || [ ! -s "$STEP3_OUT_BASE/${n}_HOLDOUT_220S/plots/07_orange_trajectory_on_pcd_clean.png" ];then
  echo "FAIL $n rc=$rc $(date -Iseconds)"|tee -a "$STEP3_OUT_BASE/status.txt"
  tail -19 "$STEP3_OUT_BASE/${n}_execution.log";exit 3
 fi
 grep -E '^reference_disagreement_3d_rmse_m=|^reference_disagreement_failure_rmse3d_m=|^step_max_m=' "$STEP3_OUT_BASE/${n}_HOLDOUT_220S/metrics_exact.txt"|tee -a "$STEP3_OUT_BASE/status.txt"
 echo "COMPLETE $n $(date -Iseconds)"|tee -a "$STEP3_OUT_BASE/status.txt"
done
python3 scripts/summarize_heldout_turn_aware_20261014.py --results "$STEP3_OUT_BASE" --reference "$STEP3_REFERENCE" --map step3_global/map_db_sweeps_fast/cache/global_map.pcd --out "$STEP3_OUT_BASE/COMPARISON" > "$STEP3_OUT_BASE/comparison_generation.log" 2>&1
rc=$?;if [ "$rc" -ne 0 ];then echo "COMPARISON_FAILED rc=$rc"|tee -a "$STEP3_OUT_BASE/status.txt";tail -25 "$STEP3_OUT_BASE/comparison_generation.log";exit 4;fi
echo 'HELDOUT_FOUR_MODES_FINISHED'|tee -a "$STEP3_OUT_BASE/status.txt"
