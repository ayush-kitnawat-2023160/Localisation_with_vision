#!/usr/bin/env bash
# AFTER three development failure tests and gate pass, freeze then one-shot heldout.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)";cd "$ROOT" || exit 2
D="$ROOT/results/camera_lidar_imu_gps/LOOP_BOUNDED_DEV_20261014"
F="$D/failure_status.txt"
mkdir -p "$D"
for n in $(seq 1 2500);do
  grep -q 'ALL_DEVELOPMENT_MODES_COMPLETE' "$F" 2>/dev/null && break
  # Explicit failed runs are never promoted.
  if grep -qE '^FAILED |^ABORT:' "$F" 2>/dev/null;then echo 'DEVELOPMENT_ABORT';exit 2;fi
  sleep 2
done
if ! grep -q 'ALL_DEVELOPMENT_MODES_COMPLETE' "$F" 2>/dev/null;then echo 'DEVELOPMENT_INCOMPLETE; HOLDOUT NOT STARTED';exit 3;fi
python3 scripts/evaluate_turn_fastcatch_failure_regressions.py \
 --base "$ROOT/results/camera_lidar_imu_gps/LATEST_20261013_1X_290S" \
 --candidate "$D" --out "$D/development_failure_quality_gate.json" \
 > "$D/development_failure_quality_gate.log" 2>&1
if [ "$?" -ne 0 ];then echo 'REGRESSION_GATE_FAILED; HELDOUT NOT RUN';tail -40 "$D/development_failure_quality_gate.log";exit 4;fi
mkdir -p "$D/FROZEN_CANDIDATE"
cp launch/fast_livo2_step3_global_turn_aware_fastcatch_dev.launch "$D/FROZEN_CANDIDATE/"
cp config/failsafe_rosbag.yaml config/calibration/recorded_cart_bag.yaml "$D/FROZEN_CANDIDATE/"
sha256sum src/step3_global_acquirer.cpp src/LIVMapper.cpp \
  launch/fast_livo2_step3_global_turn_aware_fastcatch_dev.launch \
  config/failsafe_rosbag.yaml config/calibration/recorded_cart_bag.yaml \
  > "$D/FROZEN_CANDIDATE/source_sha256.txt"
docker image inspect fast_livo2_failsafe:loop-turn-aware-dev-20261014 --format '{{.Id}}' \
 > "$D/FROZEN_CANDIDATE/image_id.txt"
printf 'FROZEN_DEVELOPMENT_GATE_PASSED %s\n' "$(date -Iseconds)" > "$D/FROZEN_CANDIDATE/status.txt"
echo 'PASS: development healthy and all three failures; heldout starts WITHOUT ANY TUNING'
./scripts/run_turn_fastcatch_holdout_20261014.sh
