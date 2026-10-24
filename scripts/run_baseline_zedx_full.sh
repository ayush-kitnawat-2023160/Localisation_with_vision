#!/usr/bin/env bash
# Full-length NORMAL FAST-LIVO2 (LIO + visual) ROS-bag sanity replay.
# Keeps outputs independent of the failsafe map build. Does not overwrite.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROSBAG_ROOT="$(cd "$ROOT/../rosbag" && pwd)"
OUT_ROOT="$ROOT/outputs"
mkdir -p "$OUT_ROOT"
RUN_NAME="${RUN_NAME:-baseline_full_20261019}"
BAG_NAME="${BAG_NAME:-localization_full.bag}"
if [[ "$BAG_NAME" == */* || ! -f "$ROSBAG_ROOT/looooo/$BAG_NAME" ]]; then
  echo "Invalid BAG_NAME: $BAG_NAME (expected a filename in rosbag/looooo/)" >&2
  exit 2
fi
OUTPUT_DIR="$OUT_ROOT/$RUN_NAME"
if [[ -e "$OUTPUT_DIR/trajectory.csv" ]]; then
  echo "Refusing to overwrite previous trajectory: $OUTPUT_DIR/trajectory.csv" >&2
  exit 2
fi
mkdir -p "$OUTPUT_DIR"
RATE="${RATE:-0.75}"
ENABLE_VISUAL="${ENABLE_VISUAL:-true}"
VISUAL_MODE="${VISUAL_MODE:-fusion}"
MAX_CAMERA_HZ="${MAX_CAMERA_HZ:-5.0}"
IMAGE="${IMAGE:-fast_livo2:dev}"
if [[ "$VISUAL_MODE" != "fusion" && "$VISUAL_MODE" != "track_only" && "$VISUAL_MODE" != "camera_lidar_scheduler" ]]; then
  echo "VISUAL_MODE must be fusion, track_only, or camera_lidar_scheduler" >&2; exit 2
fi
CONTAINER="${CONTAINER:-fastlivo_zedx_standard_baseline}"
if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "Existing container $CONTAINER; stop/remove it before another replay" >&2
  exit 2
fi
echo "FAST-LIVO2 baseline: $BAG_NAME, rosbag rate $RATE, visual=$ENABLE_VISUAL, mode=$VISUAL_MODE, camera_hz=$MAX_CAMERA_HZ"
echo "Outputs: $OUTPUT_DIR"
docker run -d --rm \
  --name "$CONTAINER" \
  --memory=7g --memory-swap=8g \
  --volume "$ROSBAG_ROOT:/data/rosbag:ro" \
  --volume "$OUT_ROOT:/data/outputs" \
  --volume "$ROOT/launch/fast_livo2_baseline_zedx_rosbag.launch:/ws/src/Fast_Livo2/launch/fast_livo2_baseline_zedx_rosbag.launch:ro" \
  --env "RATE=$RATE" --env "ENABLE_VISUAL=$ENABLE_VISUAL" --env "VISUAL_MODE=$VISUAL_MODE" --env "MAX_CAMERA_HZ=$MAX_CAMERA_HZ" \
  --env "BAG_NAME=$BAG_NAME" \
  --env "RUN_NAME=$RUN_NAME" \
  "$IMAGE" bash -lc '
    set -euo pipefail
    out="/data/outputs/$RUN_NAME"
    mkdir -p "$out"
    roscore >"$out/roscore.log" 2>&1 &
    core=$!
    launch=""
    cleanup() {
      if [[ -n "$launch" ]]; then kill -INT "$launch" 2>/dev/null || true; fi
      kill "$core" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM
    sleep 4
    roslaunch fast_livo2 fast_livo2_baseline_zedx_rosbag.launch \
      enable_visual:="$ENABLE_VISUAL" \
      visual_mode:="$VISUAL_MODE" \
      max_camera_hz:="$MAX_CAMERA_HZ" \
      trajectory_output:="$out/trajectory.csv" \
      >"$out/launch.log" 2>&1 &
    launch=$!
    sleep 5
    if ! kill -0 "$launch" 2>/dev/null; then
      echo "Baseline launch failed:"; tail -n 65 "$out/launch.log"; exit 2
    fi
    # Hard assertion: both labels must correspond to actual ROS parameters.
    # An older roslaunch Python-bool/string comparison silently set img_en=0
    # even when ENABLE_VISUAL=true, invalidating a visual-on comparison.
    image_param="$(rosparam get /common/img_en)"
    expected_image_param=0
    if [[ "$ENABLE_VISUAL" == "true" ]]; then expected_image_param=1; fi
    if [[ "$image_param" != "$expected_image_param" ]]; then
      echo "BAD BASELINE CONFIG: /common/img_en=$image_param, expected $expected_image_param" >&2
      exit 3
    fi
    track_param="$(rosparam get /vio/diagnostic_tracking_only_en)"
    expected_track_param=false
    if [[ "$VISUAL_MODE" == "track_only" ]]; then expected_track_param=true; fi
    if [[ "$track_param" != "$expected_track_param" ]]; then
      echo "BAD DIAGNOSTIC CONFIG: /vio/diagnostic_tracking_only_en=$track_param, expected $expected_track_param" >&2
      exit 3
    fi
    lidar_sync_param="$(rosparam get /vio/diagnostic_force_lidar_sync_en)"
    expected_lidar_sync_param=false
    if [[ "$VISUAL_MODE" == "camera_lidar_scheduler" ]]; then expected_lidar_sync_param=true; fi
    if [[ "$lidar_sync_param" != "$expected_lidar_sync_param" ]]; then
      echo "BAD DIAGNOSTIC CONFIG: /vio/diagnostic_force_lidar_sync_en=$lidar_sync_param, expected $expected_lidar_sync_param" >&2
      exit 3
    fi
    printf "verified_common_img_en=%s\nverified_tracking_only=%s\nverified_lidar_scheduler_only=%s\nrate=%s\ncamera_hz=%s\n" "$image_param" "$track_param" "$lidar_sync_param" "$RATE" "$MAX_CAMERA_HZ" >"$out/launch_verification.txt"
    (
      while kill -0 "$launch" 2>/dev/null; do
        printf "%s " "$(date +%s)"
        # Linux TASK_COMM_LEN truncates 'fastlivo_mapping' to
        # 'fastlivo_mappin', so ps -C fastlivo_mapping never matches.
        ps -C fastlivo_mappin -o %cpu=,rss=,etime= | head -1 || true
        sleep 2
      done
    ) >"$out/estimator_resources.txt" 2>&1 &
    monitor=$!
    echo "bagplay_start $(date -Is)" >"$out/status.txt"
    set +e
    timeout --signal=INT 1400 rosbag play --clock -r "$RATE" \
      "/data/rosbag/looooo/$BAG_NAME" \
      >"$out/rosbag_play.log" 2>&1
    rc=$?
    set -e
    echo "bagplay_exit=$rc $(date -Is)" >>"$out/status.txt"
    sleep 6
    cleanup
    kill "$monitor" 2>/dev/null || true
    trap - EXIT INT TERM
    echo "run_complete $(date -Is)" >>"$out/status.txt"
    echo "Baseline finished with rosbag exit code $rc"
    if [[ "$rc" != 0 ]]; then exit "$rc"; fi
  '
