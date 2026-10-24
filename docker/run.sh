#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROSBAG_DIR="${ROSBAG_DIR:-$PACKAGE_DIR/../rosbag}"
DATA_DIR="$(cd "$ROSBAG_DIR" && pwd)"
MAP_DIR="${MAP_DIR:-$PACKAGE_DIR/maps}"
MAP_DATA_DIR="$(cd "$MAP_DIR" && pwd)"
OUTPUT_DIR="${OUTPUT_DIR:-$PACKAGE_DIR/outputs}"
mkdir -p "$OUTPUT_DIR"
OUTPUT_DATA_DIR="$(cd "$OUTPUT_DIR" && pwd)"


DOCKER_ARGS=(
  --rm
  --interactive
  --tty
  --name fast_livo2_dev
  --volume "$DATA_DIR:/data/rosbag:ro"
  --volume "$MAP_DATA_DIR:/data/maps:ro"
  --volume "$OUTPUT_DATA_DIR:/data/outputs"
)
if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
  DOCKER_ARGS+=(--env "DISPLAY=${DISPLAY}" --volume /tmp/.X11-unix:/tmp/.X11-unix:ro)
fi

docker run "${DOCKER_ARGS[@]}" fast_livo2:dev "$@"
