#!/usr/bin/env bash
# Run ONLY after the corresponding 290s healthy gate passes. 1x ORIGINAL development bag.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)";cd "$ROOT" || exit 2
B="$ROOT/results/camera_lidar_imu_gps/LOOP_BOUNDED_DEV_20261014"
REF=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/output_image_voxel_loopclosed/optimized_trajectory.csv
export BAG_PATH=/home/devil/Desktop/car/alive-repo/rosbag/camera_lidar_imu_gps.bag STEP3_OUT_BASE="$B"
export STEP3_IMAGE=fast_livo2_failsafe:loop-turn-aware-dev-20261014
export STEP3_LAUNCH_FILE="$ROOT/launch/fast_livo2_step3_global_turn_aware_dev.launch"
export STEP3_CONFIG="$ROOT/config/failsafe_rosbag.yaml"
export STEP3_CALIBRATION_FILE="$ROOT/config/calibration/recorded_cart_bag.yaml"
export STEP3_REFERENCE="$REF"
export STEP3_VISUAL_MAP=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/final_failsafe_maps/final_camera_rgb_lidar_voxel_15cm.pcd
export STEP3_VISUAL_MAP_IMAGES=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/output_image3d_map_complete_v3/livo/image
export STEP3_VISUAL_MAP_TRAJ=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/output_image_voxel_loopclosed/optimized_trajectory.csv
export STEP3_VISUAL_PLACE_ENABLE=true STEP3_LIDAR_FEATURE_PNP=false STEP3_LIDAR_AKAZE=false
export STEP3_TRACKING_ESIKF_MEASUREMENT_ENABLE=false STEP3_GYRO_BIAS_FEEDBACK_ENABLE=false STEP3_ANCHORED_LOCAL_FEEDBACK_ENABLE=false STEP3_CONTINUITY_RETRY_ENABLE=true
export PLAY_DURATION=290 PLAY_RATE=1 DROP_START=125 DROP_DURATION=40
for n in $(seq 1 450);do
  test -s "$B/HEALTHY_TURN_AWARE_290S/metrics_exact.txt" && test -s "$B/HEALTHY_TURN_AWARE_290S/plots/07_orange_trajectory_on_pcd_clean.png" && break
  sleep 1
done
if ! test -s "$B/HEALTHY_TURN_AWARE_290S/plots/07_orange_trajectory_on_pcd_clean.png";then echo "ABORT: bounded healthy test did not finish";exit 2;fi
python3 scripts/evaluate_loop_continuity_candidate.py \
  --baseline results/camera_lidar_imu_gps/LATEST_20261013_1X_290S/HEALTHY \
  --candidate "$B/HEALTHY_TURN_AWARE_290S" \
  --reference "$REF" --out "$B/healthy_turn_quality_gate.json" > "$B/healthy_turn_quality_gate.log" 2>&1
if [ "$?" -ne 0 ];then echo 'ABORT: healthy development gate failed';tail -25 "$B/healthy_turn_quality_gate.log";exit 2;fi
printf 'START_DEVELOPMENT_FAILURE_MODES %s\n' "$(date -Iseconds)" > "$B/failure_status.txt"
for case in CAMERA:camera LIDAR:lidar IMU:imu;do
 n="${case%%:*}";s="${case#*:}"
 echo "BEGIN $n $(date -Iseconds)"|tee -a "$B/failure_status.txt"
 bash ./scripts/run_step3_global_case.sh "${n}_FAIL_TURN_AWARE_290S" "$s" > "$B/${n}_FAIL_TURN_AWARE_execution.log" 2>&1
 rc=$?
 if [ "$rc" -ne 0 ] || [ ! -s "$B/${n}_FAIL_TURN_AWARE_290S/metrics_exact.txt" ] || [ ! -s "$B/${n}_FAIL_TURN_AWARE_290S/plots/07_orange_trajectory_on_pcd_clean.png" ];then
  echo "FAILED $n rc=$rc $(date -Iseconds)"|tee -a "$B/failure_status.txt";tail -18 "$B/${n}_FAIL_TURN_AWARE_execution.log";exit 3
 fi
 grep -E '^reference_disagreement_3d_rmse_m=|^reference_disagreement_failure_rmse3d_m=|^step_max_m=' "$B/${n}_FAIL_TURN_AWARE_290S/metrics_exact.txt" |tee -a "$B/failure_status.txt"
 echo "COMPLETE $n $(date -Iseconds)"|tee -a "$B/failure_status.txt"
done
python3 scripts/report_development_bag_four_modes.py --reference "$REF" \
  --map-pcd step3_global/map_db_sweeps_fast/cache/global_map.pcd \
  --out "$B/COMPARISON_TURN_AWARE" --healthy "$B/HEALTHY_TURN_AWARE_290S" \
  --camera "$B/CAMERA_FAIL_TURN_AWARE_290S" --lidar "$B/LIDAR_FAIL_TURN_AWARE_290S" \
  --imu "$B/IMU_FAIL_TURN_AWARE_290S" > "$B/COMPARISON_TURN_AWARE.log" 2>&1
rc=$?;if [ "$rc" -ne 0 ];then echo COMPARISON_FAILED;tail -16 "$B/COMPARISON_TURN_AWARE.log";exit 4;fi
echo 'ALL_DEVELOPMENT_MODES_COMPLETE'|tee -a "$B/failure_status.txt"
