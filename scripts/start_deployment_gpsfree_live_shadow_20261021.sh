#!/usr/bin/env bash
# On ROS Noetic target NUC OR preconfigured ROS1 docker host: SHADOW ONLY.
# Requires CALIBRATION=/measured.yaml MAP_DB=/surveyed/mapdb
# and VISUAL_MAP_DIR=/visual/map. No autonomous navigation or /pose.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${CALIBRATION:?Supply CALIBRATION=/absolute/path/to/measured.yaml, NEVER the archived offline profile}"
: "${MAP_DB:?Supply MAP_DB=/absolute/path/to/map_database}"
: "${VISUAL_MAP_DIR:?Supply VISUAL_MAP_DIR=/absolute/path/to/visual_map_root}"
: "${LIDAR_POINT_TIME_UNIT:?Explicitly supply LIDAR_POINT_TIME_UNIT=seconds or microseconds after checking the driver}"
case "$LIDAR_POINT_TIME_UNIT" in seconds|microseconds) ;; *) echo 'Invalid LiDAR driver point-time unit' >&2; exit 2;;esac
VISUAL="${VISUAL:-true}"
case "$VISUAL" in true|false) ;; *)echo 'VISUAL must be true/false' >&2;exit 2;;esac
if [[ "$CALIBRATION" != /* || "$MAP_DB" != /* || "$VISUAL_MAP_DIR" != /* ]];then
  echo 'All calibration/map paths must be absolute; ambiguous relative paths refused' >&2
  exit 2
fi
worker_args=()
if [[ "$VISUAL" == true ]];then
  : "${LEARNED_MAP_DIR:?Set LEARNED_MAP_DIR=/absolute/physically_validated_learned_visual_map when VISUAL=true}"
  [[ "$LEARNED_MAP_DIR" == /* ]] || { echo 'LEARNED_MAP_DIR must be absolute';exit 2; }
  worker_args=(--require-visual-worker --learned-map "$LEARNED_MAP_DIR")
fi
python3 "$ROOT/scripts/preflight_gpsfree_deployment_20261021.py" \
  --mode live --calibration "$CALIBRATION" --lidar-map "$MAP_DB" \
  --visual-map "$VISUAL_MAP_DIR" "${worker_args[@]}"
if [[ "${EXECUTE:-false}" != true ]];then
  echo
  echo 'PREFLIGHT PASS, DRY-RUN ONLY. Use EXECUTE=true ONLY once the NUC,'
  echo 'Orin data topics, physical measured calibration and independent vehicle'
  echo 'stop systems have been reviewed. Still no autonomous pose authority.'
  echo
  printf 'roslaunch fast_livo2 deployment_gpsfree_shadow_20261021.launch use_sim_time:=false calibration_file:=%q map_db:=%q visual_map_dir:=%q lidar_point_time_unit:=%q learned_visual_enable:=%s\n' \
       "$CALIBRATION" "$MAP_DB" "$VISUAL_MAP_DIR" "$LIDAR_POINT_TIME_UNIT" "$VISUAL"
  exit 0
fi
if ! command -v roslaunch >/dev/null;then
  echo 'ROS Noetic roslaunch not installed in this shell; use the Noetic container/environment' >&2
  exit 3
fi
echo 'STARTING NON-AUTHORITATIVE LIVE SHADOW: no /pose, no motor/stop command, no automatic failover'
exec roslaunch fast_livo2 deployment_gpsfree_shadow_20261021.launch \
  use_sim_time:=false calibration_file:="$CALIBRATION" \
  map_db:="$MAP_DB" visual_map_dir:="$VISUAL_MAP_DIR" \
  lidar_point_time_unit:="$LIDAR_POINT_TIME_UNIT" \
  learned_visual_enable:="$VISUAL" \
  candidate_fused_map_pose_enable:=true
