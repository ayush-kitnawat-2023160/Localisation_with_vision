#!/usr/bin/env bash
# Experimental only: development 1x rosbag, all four modes, then strict gates.
# NEVER promote to the cart from an unchecked single-mode result.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
D="$ROOT/results/camera_lidar_imu_gps/RGBD_FRESHNESS_HANDOFF_DEV_20261014"
BASE="$ROOT/results/camera_lidar_imu_gps/LATEST_20261013_1X_290S"
REF="/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/output_image_voxel_loopclosed/optimized_trajectory.csv"
export BAG_PATH=/home/devil/Desktop/car/alive-repo/rosbag/camera_lidar_imu_gps.bag
export STEP3_OUT_BASE="$D"
export STEP3_IMAGE=fast_livo2_failsafe:rgbd-freshness-handoff-dev-20261014
export STEP3_LAUNCH_FILE="$ROOT/launch/fast_livo2_step3_global_map_motion_relock_dev.launch"
export STEP3_CONFIG="$ROOT/config/failsafe_rosbag_rgbd_freshness_dev.yaml"
export STEP3_CALIBRATION_FILE="$ROOT/config/calibration/recorded_cart_bag.yaml"
export STEP3_REFERENCE="$REF"
export STEP3_VISUAL_MAP=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/final_failsafe_maps/final_camera_rgb_lidar_voxel_15cm.pcd
export STEP3_VISUAL_MAP_IMAGES=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/output_image3d_map_complete_v3/livo/image
export STEP3_VISUAL_MAP_TRAJ="$REF"
export STEP3_VISUAL_PLACE_ENABLE=true STEP3_LIDAR_FEATURE_PNP=false STEP3_LIDAR_AKAZE=false
export STEP3_TRACKING_ESIKF_MEASUREMENT_ENABLE=false STEP3_GYRO_BIAS_FEEDBACK_ENABLE=false
export STEP3_ANCHORED_LOCAL_FEEDBACK_ENABLE=false STEP3_CONTINUITY_RETRY_ENABLE=true
export PLAY_DURATION=290 PLAY_RATE=1 DROP_START=125 DROP_DURATION=40
mkdir -p "$D"

for spec in 'LIDAR_FAIL_RGBDFRESH_290S:lidar:LIDAR_FAIL' \
            'IMU_FAIL_RGBDFRESH_290S:imu:IMU_FAIL' \
            'HEALTHY_RGBDFRESH_290S:none:HEALTHY' \
            'CAMERA_FAIL_RGBDFRESH_290S:camera:CAMERA_FAIL'; do
  IFS=: read -r name sensor previous <<< "$spec"
  echo "BEGIN $name $(date -Iseconds)" | tee -a "$D/validation_status.txt"
  if [[ ! -s "$D/$name/metrics_exact.txt" || \
        ! -s "$D/$name/plots/07_orange_trajectory_on_pcd_clean.png" ]]; then
    ./scripts/run_step3_global_case.sh "$name" "$sensor" > "$D/$name.execution.log" 2>&1 || {
      echo "RUN_FAILED $name" | tee -a "$D/validation_status.txt"
      tail -35 "$D/$name.execution.log"
      exit 2
    }
  fi
  if [[ "$sensor" == none ]]; then
    python3 scripts/evaluate_loop_continuity_candidate.py \
      --baseline "$BASE/HEALTHY" --candidate "$D/$name" \
      --reference "$REF" --out "$D/$name.quality_gate.json" \
      > "$D/$name.quality_gate.log" 2>&1 || {
        echo "HEALTHY_GATE_FAILED" | tee -a "$D/validation_status.txt"
        tail -35 "$D/$name.quality_gate.log"
        exit 3
      }
  else
    python3 scripts/evaluate_recovery_catchup_gate.py \
      --baseline "$BASE/$previous" --candidate "$D/$name" --case "$sensor" \
      --out "$D/$name.quality_gate.json" \
      > "$D/$name.quality_gate.log" 2>&1 || {
        echo "RECOVERY_GATE_FAILED $name" | tee -a "$D/validation_status.txt"
        tail -35 "$D/$name.quality_gate.log"
        exit 4
      }
  fi
  echo "PASS $name $(date -Iseconds)" | tee -a "$D/validation_status.txt"
done
python3 scripts/report_development_bag_four_modes.py \
  --reference "$REF" \
  --map-pcd step3_global/map_db_sweeps_fast/cache/global_map.pcd \
  --out "$D/COMPARISON" \
  --healthy "$D/HEALTHY_RGBDFRESH_290S" \
  --camera "$D/CAMERA_FAIL_RGBDFRESH_290S" \
  --lidar "$D/LIDAR_FAIL_RGBDFRESH_290S" \
  --imu "$D/IMU_FAIL_RGBDFRESH_290S" > "$D/comparison.log" 2>&1
mkdir -p "$D/FROZEN_CANDIDATE"
cp "$STEP3_LAUNCH_FILE" "$STEP3_CONFIG" "$STEP3_CALIBRATION_FILE" "$D/FROZEN_CANDIDATE/"
sha256sum src/step3_global_acquirer.cpp src/LIVMapper.cpp \
  "$STEP3_LAUNCH_FILE" "$STEP3_CONFIG" "$STEP3_CALIBRATION_FILE" \
  > "$D/FROZEN_CANDIDATE/source_sha256.txt"
docker image inspect "$STEP3_IMAGE" --format '{{.Id}}' \
  > "$D/FROZEN_CANDIDATE/image_id.txt"
echo "FROZEN_DEV_ALL_FOUR_PASSED $(date -Iseconds)" | tee -a "$D/validation_status.txt"
echo 'Development passed. Heldout must be run with exactly this frozen configuration.'
