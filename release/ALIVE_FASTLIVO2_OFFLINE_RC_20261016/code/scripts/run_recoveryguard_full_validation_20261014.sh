#!/usr/bin/env bash
# Development diagnostic -> complete development regression -> freeze -> held-out.
# No tuning or September inspection occurs inside this script.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)";cd "$ROOT" || exit 2
D="$ROOT/results/camera_lidar_imu_gps/LOOP_RECOVERYGUARD_DEV_20261014"
REF=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/output_image_voxel_loopclosed/optimized_trajectory.csv
mkdir -p "$D"
for _ in $(seq 1 450);do
  test -s "$D/LIDAR_FAIL_DIAG_210S/metrics_exact.txt" && test -s "$D/LIDAR_FAIL_DIAG_210S/plots/07_orange_trajectory_on_pcd_clean.png" && break
  sleep 1
done
if [ ! -s "$D/LIDAR_FAIL_DIAG_210S/plots/07_orange_trajectory_on_pcd_clean.png" ];then echo 'DIAGNOSTIC INCOMPLETE';exit 2;fi
python3 scripts/evaluate_lidar_recovery_210s.py --baseline "$ROOT/results/camera_lidar_imu_gps/LATEST_20261013_1X_290S/LIDAR_FAIL" --trial "$D/LIDAR_FAIL_DIAG_210S" --reference "$REF" --out "$D/lidar_recovery_diagnostic_gate.json" > "$D/lidar_recovery_diagnostic_gate.log" 2>&1
rc=$?;if [ "$rc" -ne 0 ];then echo 'LIDAR RECOVERY DIAGNOSTIC FAILED; no next phase';tail -35 "$D/lidar_recovery_diagnostic_gate.log";exit 3;fi
export BAG_PATH=/home/devil/Desktop/car/alive-repo/rosbag/camera_lidar_imu_gps.bag STEP3_OUT_BASE="$D"
export STEP3_IMAGE=fast_livo2_failsafe:loop-turn-recoveryguard-20261014
export STEP3_LAUNCH_FILE="$ROOT/launch/fast_livo2_step3_global_recovery_quarantine_dev.launch"
export STEP3_CONFIG="$ROOT/config/failsafe_rosbag.yaml" STEP3_CALIBRATION_FILE="$ROOT/config/calibration/recorded_cart_bag.yaml"
export STEP3_REFERENCE="$REF"
export STEP3_VISUAL_MAP=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/final_failsafe_maps/final_camera_rgb_lidar_voxel_15cm.pcd
export STEP3_VISUAL_MAP_IMAGES=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/output_image3d_map_complete_v3/livo/image
export STEP3_VISUAL_MAP_TRAJ=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/output_image_voxel_loopclosed/optimized_trajectory.csv
export STEP3_VISUAL_PLACE_ENABLE=true STEP3_LIDAR_FEATURE_PNP=false STEP3_LIDAR_AKAZE=false
export STEP3_TRACKING_ESIKF_MEASUREMENT_ENABLE=false STEP3_GYRO_BIAS_FEEDBACK_ENABLE=false STEP3_ANCHORED_LOCAL_FEEDBACK_ENABLE=false STEP3_CONTINUITY_RETRY_ENABLE=true
export PLAY_DURATION=290 PLAY_RATE=1 DROP_START=125 DROP_DURATION=40
printf 'LIDAR_RECOVERY_DIAG_PASS %s\n' "$(date -Iseconds)" > "$D/full_run_status.txt"
for case in HEALTHY:none CAMERA_FAIL:camera LIDAR_FAIL:lidar IMU_FAIL:imu;do
 name="${case%%:*}";mode="${case#*:}"
 folder="${name}_TURN_FASTCATCH_290S"
 echo "START $folder $(date -Iseconds)"|tee -a "$D/full_run_status.txt"
 ./scripts/run_step3_global_case.sh "$folder" "$mode" > "$D/${name}_full_execution.log" 2>&1
 rc=$?
 if [ "$rc" -ne 0 ] || [ ! -s "$D/$folder/metrics_exact.txt" ] || [ ! -s "$D/$folder/plots/07_orange_trajectory_on_pcd_clean.png" ];then
  echo "FAIL $folder rc=$rc"|tee -a "$D/full_run_status.txt";tail -18 "$D/${name}_full_execution.log";exit 4
 fi
 grep -E '^reference_disagreement_3d_rmse_m=|^reference_disagreement_failure_rmse3d_m=|^step_max_m=' "$D/$folder/metrics_exact.txt"|tee -a "$D/full_run_status.txt"
 echo "COMPLETE $folder $(date -Iseconds)"|tee -a "$D/full_run_status.txt"
done
python3 scripts/evaluate_loop_continuity_candidate.py --baseline "$ROOT/results/camera_lidar_imu_gps/LATEST_20261013_1X_290S/HEALTHY" --candidate "$D/HEALTHY_TURN_FASTCATCH_290S" --reference "$REF" --out "$D/healthy_quality_gate.json" > "$D/healthy_quality_gate.log" 2>&1
if [ "$?" -ne 0 ];then echo 'FULL HEALTHY DEVELOPMENT GATE FAILED; HELDOUT ABORTED';tail -23 "$D/healthy_quality_gate.log";exit 5;fi
python3 scripts/evaluate_turn_fastcatch_failure_regressions.py --base "$ROOT/results/camera_lidar_imu_gps/LATEST_20261013_1X_290S" --candidate "$D" --out "$D/development_failure_quality_gate.json" > "$D/development_failure_quality_gate.log" 2>&1
if [ "$?" -ne 0 ];then echo 'FAILURE REGRESSION GATE FAILED; HELDOUT ABORTED';tail -33 "$D/development_failure_quality_gate.log";exit 6;fi
python3 scripts/report_development_bag_four_modes.py --reference "$REF" --map-pcd step3_global/map_db_sweeps_fast/cache/global_map.pcd --out "$D/COMPARISON" --healthy "$D/HEALTHY_TURN_FASTCATCH_290S" --camera "$D/CAMERA_FAIL_TURN_FASTCATCH_290S" --lidar "$D/LIDAR_FAIL_TURN_FASTCATCH_290S" --imu "$D/IMU_FAIL_TURN_FASTCATCH_290S" > "$D/comparison.log" 2>&1
if [ "$?" -ne 0 ];then echo 'DEVELOPMENT PLOT GENERATION FAILED';tail -19 "$D/comparison.log";exit 7;fi
mkdir -p "$D/FROZEN_CANDIDATE"
cp launch/fast_livo2_step3_global_recovery_quarantine_dev.launch "$D/FROZEN_CANDIDATE/"
cp config/failsafe_rosbag.yaml config/calibration/recorded_cart_bag.yaml "$D/FROZEN_CANDIDATE/"
sha256sum src/step3_global_acquirer.cpp src/LIVMapper.cpp launch/fast_livo2_step3_global_recovery_quarantine_dev.launch config/failsafe_rosbag.yaml config/calibration/recorded_cart_bag.yaml > "$D/FROZEN_CANDIDATE/source_sha256.txt"
docker image inspect fast_livo2_failsafe:loop-turn-recoveryguard-20261014 --format '{{.Id}}' > "$D/FROZEN_CANDIDATE/image_id.txt"
echo 'FROZEN_DEVELOPMENT_GATE_PASSED'|tee -a "$D/full_run_status.txt"
./scripts/run_recoveryguard_holdout_20261014.sh
