#!/usr/bin/env bash
# Reproducible original development bag validation. NOT deployment certification.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT" || exit 2
BAG=/home/devil/Desktop/car/alive-repo/rosbag/camera_lidar_imu_gps.bag
REF=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2/output_image_voxel_loopclosed/optimized_trajectory.csv
MAPBASE=/home/devil/Desktop/car/alive-repo/old_tests/temp_fast_livo_2
OUT="$ROOT/results/camera_lidar_imu_gps/LATEST_20261013_1X_290S"
mkdir -p "$OUT"
export BAG_PATH="$BAG" STEP3_OUT_BASE="$OUT"
export STEP3_IMAGE=fast_livo2_failsafe:dev
export STEP3_CONFIG="$ROOT/config/failsafe_rosbag.yaml"
export STEP3_CALIBRATION_FILE="$ROOT/config/calibration/recorded_cart_bag.yaml"
export STEP3_LAUNCH_FILE="$ROOT/launch/fast_livo2_step3_global_rosbag.launch"
export STEP3_REFERENCE="$REF"
export STEP3_VISUAL_MAP="$MAPBASE/final_failsafe_maps/final_camera_rgb_lidar_voxel_15cm.pcd"
export STEP3_VISUAL_MAP_IMAGES="$MAPBASE/output_image3d_map_complete_v3/livo/image"
export STEP3_VISUAL_MAP_TRAJ="$MAPBASE/output_image_voxel_loopclosed/optimized_trajectory.csv"
export STEP3_VISUAL_PLACE_ENABLE=true
export STEP3_LIDAR_FEATURE_PNP=false STEP3_LIDAR_AKAZE=false
export STEP3_LIO_GUARD_ENABLE=false
export STEP3_TRACKING_ESIKF_MEASUREMENT_ENABLE=false
export STEP3_GYRO_BIAS_FEEDBACK_ENABLE=false
export STEP3_ANCHORED_LOCAL_FEEDBACK_ENABLE=false
export PLAY_DURATION=290 PLAY_RATE=1 DROP_START=125 DROP_DURATION=40
cat > "$OUT/README.md" <<'EOF'
# Latest four-mode development bag replay — 2026-10-13

**Dataset:** original `camera_lidar_imu_gps.bag` (not 18 September). 290 s per case, 1× playback.

**Sensor failures:** one sensor fails for the 125–165 s interval of each non-healthy replay.

**Runtime:** preserved `fast_livo2_failsafe:dev`; original calibration; 15cm RGB/LiDAR saved voxel map; automatic global PCD localization; visual place hints enabled; experimental map-to-ESIKF and gyro-bias feedback disabled.

**Important:** Saved map/loop-closed trajectory comparisons are **proxy disagreement, not independent ground truth**. These plots do not certify safe autonomy. Healthy estimator may drift even when the global output is PCD corrected.

**Archive:** old results moved to `../../../results_archive/20261013_pre_latest_runs/` and are preserved, not deleted.

**Artifacts expected:** each of `HEALTHY`, `CAMERA_FAIL`, `LIDAR_FAIL`, `IMU_FAIL` gets `plots/07_orange_trajectory_on_pcd_clean.png`, other graphs, run metadata and evaluation metrics; `COMPARISON/` gets combined plots and CSV.

**Run status:** in progress; see `run_status.txt` for per-case outcomes.
EOF
: > "$OUT/run_status.txt"
failures=0
for spec in HEALTHY:none CAMERA_FAIL:camera LIDAR_FAIL:lidar IMU_FAIL:imu;do
  name="${spec%%:*}";sensor="${spec#*:}"
  printf '[%s] START %s sensor=%s\n' "$(date --iso-8601=seconds)" "$name" "$sensor" | tee -a "$OUT/run_status.txt"
  bash "$ROOT/scripts/run_step3_global_case.sh" "$name" "$sensor" > "$OUT/${name}_execution.log" 2>&1
  rc=$?
  test -f "$OUT/$name/metrics_exact.txt" || rc=81
  test -s "$OUT/$name/plots/07_orange_trajectory_on_pcd_clean.png" || rc=82
  if [ "$rc" -ne 0 ];then
    printf '[%s] FAILED %s rc=%s\n' "$(date --iso-8601=seconds)" "$name" "$rc" | tee -a "$OUT/run_status.txt"
    tail -18 "$OUT/${name}_execution.log"
    failures=$((failures+1))
  else
    printf '[%s] COMPLETE %s\n' "$(date --iso-8601=seconds)" "$name" | tee -a "$OUT/run_status.txt"
    grep -E '^(reference_disagreement_3d_rmse_m|step_max_m|reference_disagreement_failure_rmse3d_m)=' "$OUT/$name/metrics_exact.txt" | tee -a "$OUT/run_status.txt" || true
  fi
done
if [ "$failures" -eq 0 ];then
  printf '[%s] GENERATING COMPARISON\n' "$(date --iso-8601=seconds)" | tee -a "$OUT/run_status.txt"
  python3 "$ROOT/scripts/report_development_bag_four_modes.py" \
    --reference "$REF" --map-pcd "$ROOT/step3_global/map_db_sweeps_fast/cache/global_map.pcd" \
    --out "$OUT/COMPARISON" \
    --healthy "$OUT/HEALTHY" --camera "$OUT/CAMERA_FAIL" \
    --lidar "$OUT/LIDAR_FAIL" --imu "$OUT/IMU_FAIL" \
    > "$OUT/comparison_generation.log" 2>&1
  rc=$?
  if [ "$rc" -eq 0 ];then
    echo "ALL_FOUR_MODES_PLOTTED" | tee -a "$OUT/run_status.txt"
    sed -i 's/\*\*Run status:\*\* in progress; see `run_status.txt` for per-case outcomes./**Run status:** four mode runs complete and combined plots generated. See `run_status.txt` and `COMPARISON\/comparison.csv`./' "$OUT/README.md"
  else
    echo "COMPARISON_PLOTTING_FAILED rc=$rc" | tee -a "$OUT/run_status.txt"
    tail -18 "$OUT/comparison_generation.log"
    failures=$((failures+1))
  fi
else
  echo "SOME_RUNS_INCOMPLETE count=$failures" | tee -a "$OUT/run_status.txt"
fi
printf '\nSUMMARY %s\n' "$(date --iso-8601=seconds)"
cat "$OUT/run_status.txt"
exit "$failures"
