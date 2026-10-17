#!/usr/bin/env bash
# Eight 1x simulations from ONE immutable image: main+Sep18, 4 cases each.
# Deliberately FAIL CLOSED on missing data; NEVER deploys, controls or edits DBW.
set -euo pipefail
R="$(cd "$(dirname "$0")/.." && pwd)"
C="$R/results/FINAL_SINGLE_IMAGE_RC_20261016"
A="$R/step3_global/healthy_validation_assets_20261014"
H="$R/results/18_sep_2026_combined/REFERENCE_REBUILT_FROM_LIDAR_PCD_20261016"
mkdir -p "$C"
cd "$R"
export STEP3_IMAGE="fast_livo2_failsafe:single-build-rc-20261016"
export STEP3_LAUNCH_FILE="$R/launch/fast_livo2_step3_global_healthy_loop_bridge_dev.launch"
export STEP3_CONFIG="$R/config/failsafe_rosbag.yaml"
export STEP3_CALIBRATION_FILE="$R/config/calibration/recorded_cart_bag.yaml"
export STEP3_MAP_LOCALIZER_CONFIG="$R/config/map_failsafe_recorded_cart.yaml"
export STEP3_PRIOR_MAP="$R/step3_global/map_db_sweeps_fast/cache/global_map.pcd"
export STEP3_VISUAL_MAP="$A/final_camera_rgb_lidar_voxel_15cm.pcd"
export STEP3_VISUAL_MAP_IMAGES="$A/visual_map_images"
export STEP3_VISUAL_MAP_TRAJ="$A/loopclosed_reference_recovered_equivalent.csv"
export STEP3_ORB_CACHE="$A/orb_cache_dev_20261015/visual_orb_retrieval_exact.npz"
export STEP3_VISUAL_PLACE_ENABLE=true STEP3_LIDAR_FEATURE_PNP=false STEP3_LIDAR_AKAZE=false
export STEP3_TRACKING_ESIKF_MEASUREMENT_ENABLE=false STEP3_GYRO_BIAS_FEEDBACK_ENABLE=false
export STEP3_ANCHORED_LOCAL_FEEDBACK_ENABLE=false STEP3_DEFERRED_SCAN_ENABLE=true
export STEP3_CONTINUITY_RETRY_ENABLE=true
export PLAY_RATE=1 DROP_DURATION=40
ACTUAL="$(docker image inspect --format '{{.Id}}' "$STEP3_IMAGE")"
PINNED="$(cat "$C/frozen_docker_image_id.txt")"
[[ "$ACTUAL" == "$PINNED" ]] || { echo "Image changed; refusing run" >&2; exit 2; }
STATUS="$C/RUN_STATUS.csv"
if [[ -e "$STATUS" ]]; then
 echo 'Refusing to overwrite existing validation status (release append is unsafe)' >&2
 exit 2
fi
printf 'recording,mode,state,started,finished,image_id\n' > "$STATUS"
while read -r recording mode sensor; do
 [[ -n "$recording" ]] || continue
 case "$recording" in
  main)
    export BAG_PATH="/home/devil/Desktop/car/alive-repo/rosbag/camera_lidar_imu_gps.bag"
    export STEP3_OUT_BASE="$C/main"
    export STEP3_REFERENCE="$A/loopclosed_reference_recovered_equivalent.csv"
    export PLAY_DURATION=290 DROP_START=125;;
  sept18)
    export BAG_PATH="/home/devil/Desktop/car/alive-repo/rosbag/18_sep_2026_combined.bag"
    export STEP3_OUT_BASE="$C/sept18"
    export STEP3_REFERENCE="$H/REFERENCE_18SEP_LIDAR_PCD_REBUILT_20261016.csv"
    export PLAY_DURATION=220 DROP_START=90;;
  *) exit 2;;
 esac
 mkdir -p "$STEP3_OUT_BASE"
 CASE="$mode"_SINGLE_IMAGE_1X
 OUT="$STEP3_OUT_BASE/$CASE"
 if [[ -e "$OUT" ]]; then echo "Refusing to overwrite existing $OUT" >&2; exit 2; fi
 BEGIN="$(date -Iseconds)"
 echo "[FINAL RC] BEGIN $recording $mode $BEGIN image=$ACTUAL" | tee -a "$C/full_suite.log"
 printf '%s,%s,%s,%s,,%s\n' "$recording" "$mode" 'RUNNING' "$BEGIN" "$ACTUAL" >> "$STATUS"
 if ! bash "$R/scripts/run_step3_global_case.sh" "$CASE" "$sensor" > "$STEP3_OUT_BASE/$CASE.execution.log" 2>&1; then
   echo "[FINAL RC] FAILED $recording $mode" | tee -a "$C/full_suite.log"
   tail -30 "$STEP3_OUT_BASE/$CASE.execution.log" | tee -a "$C/full_suite.log"
   printf '%s,%s,%s,%s,%s,%s\n' "$recording" "$mode" 'FAILED' "$BEGIN" "$(date -Iseconds)" "$ACTUAL" >> "$STATUS"
   exit 3
 fi
 [[ -s "$OUT/global_pose.csv" && -s "$OUT/metrics_exact.txt" && -s "$OUT/plots/07_orange_trajectory_on_pcd_clean.png" ]] || {
   echo "[FINAL RC] Missing data in $OUT" >&2;exit 4; }
 echo "[FINAL RC] COMPLETE $recording $mode $(date -Iseconds)" | tee -a "$C/full_suite.log"
 grep -E '^reference_disagreement_3d_rmse_m=|^reference_disagreement_failure_rmse3d_m=|^step_max_m=|^duration=' "$OUT/metrics_exact.txt" | tee -a "$C/full_suite.log"
 printf '%s,%s,%s,%s,%s,%s\n' "$recording" "$mode" 'COMPLETE' "$BEGIN" "$(date -Iseconds)" "$ACTUAL" >> "$STATUS"
done <<'EOF'
main HEALTHY none
main CAMERA_FAIL camera
main LIDAR_FAIL lidar
main IMU_FAIL imu
sept18 HEALTHY none
sept18 CAMERA_FAIL camera
sept18 LIDAR_FAIL lidar
sept18 IMU_FAIL imu
EOF
echo '[FINAL RC] EIGHT_REPLAYS_FINISHED' | tee -a "$C/full_suite.log"
