#!/usr/bin/env bash
# Separate LOCALHOST XFeat service for archived ROS1 development bags.
# Inference produces visual place CANDIDATES only, never /pose.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-fast_livo2:learned_visual_cpu_20261020}"
PORT="${PORT:-18091}"
case "$PORT" in 18091) ;; *)echo 'For this validated offline profile use localhost port 18091' >&2;exit 2;;esac
LEARNED="$ROOT/maps/campus_full_visual_xfeat_226s_20261020"
ORB="$ROOT/maps/campus_full_visual_rgbd_226s_20261020"
CAL="$ROOT/config/calibration/zedx_current.yaml"
VENDOR="$ROOT/outputs/learned_visual_20261020/vendor"
for file in "$LEARNED/visual/keyframes.csv" "$ORB/visual/keyframes.csv" "$CAL" "$VENDOR/accelerated_features/weights/xfeat.pt";do
  test -f "$file" || { echo "Missing learned visual dependency: $file" >&2;exit 3; }
done
if ss -lnt "( sport = :$PORT )"|grep -q "127.0.0.1:$PORT";then
  echo "Port $PORT is already occupied; refusing to start duplicate" >&2;exit 4
fi
echo 'Starting LOCALHOST-only XFeat worker: OFFLINE CANDIDATE, GPS disabled.'
echo 'Use a second terminal to run raw ROS1 bag replay. Stop using Ctrl-C.'
exec docker run --rm --network host --memory=6g --name "${WORKER_NAME:-gpsfree_xfeat_offline_worker}" \
 -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=2 \
 -v "$ROOT:/repo:ro" --entrypoint bash "$IMAGE" -lc '
 cd /repo
 exec python3 -u /repo/scripts/serve_learned_visual_global.py \
   --map /repo/maps/campus_full_visual_xfeat_226s_20261020 \
   --orb-map /repo/maps/campus_full_visual_rgbd_226s_20261020 \
   --vendor /repo/outputs/learned_visual_20261020/vendor \
   --calibration /repo/config/calibration/zedx_current.yaml \
   --backend xfeat_mnn --device cpu --port 18091 \
   --retrieval-mode learned_float_flann
 '
