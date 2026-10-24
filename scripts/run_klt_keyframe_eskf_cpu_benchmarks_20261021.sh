#!/usr/bin/env bash
# Offline process CPU/RAM benchmark, separate process per estimator.
# Preserves experimental artifacts; never publishes ROS topics.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/outputs/rgbd_fallback_benchmark_20261021"
for BAG in localization_3 localization_from2;do
  for MODE in baseline keyframe_eskf;do
    NAME="klt_vs_eskf_individual_${BAG}_${MODE}_20261021"
    test ! -e "$OUT/$NAME" || { echo "Refusing overwrite $NAME";exit 2; }
    echo "=== START $BAG $MODE $(date -Ins) ==="
    docker run --rm --name "klt_eskf_${MODE}_${BAG}_20261021" \
      --entrypoint bash -v "$ROOT:/repo:rw" \
      -v "$ROOT/../rosbag/looooo:/bags:ro" fast_livo2:dev -lc \
      "set -eo pipefail; source /opt/ros/noetic/setup.bash; PYTHONPATH=/repo/scripts:\$PYTHONPATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 /repo/scripts/benchmark_klt_keyframe_eskf_20261021.py --mode '$MODE' --bag '/bags/$BAG.bag' --reference '/repo/outputs/lidar_global_holdout_20261019/${BAG}_lidar_only_odom.bag' --calibration /repo/config/calibration/zedx_current.yaml --output '/repo/outputs/rgbd_fallback_benchmark_20261021/$NAME' --duration 70 --outage-start 20 --outage-end 55" \
      >"$OUT/$NAME.log" 2>&1
    echo "=== DONE $BAG $MODE $(date -Ins) ==="
  done
done
