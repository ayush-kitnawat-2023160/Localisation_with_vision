#!/usr/bin/env bash
# Read-only ROS replay resource sampling every 30s; no health claim if data absent.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DIR="$ROOT/results/FINAL_SINGLE_IMAGE_RC_20261016"
mkdir -p "$DIR"
CSV="$DIR/docker_resource_samples.csv"
printf 'timestamp,container,cpu_percent,memory_usage,memory_percent,network_io\n' > "$CSV"
while true; do
 NAME="$(docker ps --format '{{.Names}}' | grep '^fastlivo_step3_.*_SINGLE_IMAGE_1X$' | head -1 || true)"
 if [[ -n "$NAME" ]]; then
  ROW="$(docker stats --no-stream --format '{{.CPUPerc}},{{.MemUsage}},{{.MemPerc}},{{.NetIO}}' "$NAME" 2>/dev/null | head -1 || true)"
  if [[ -n "$ROW" ]]; then echo "$(date -Iseconds),$NAME,$ROW" >> "$CSV"; fi
 fi
 if grep -q 'EIGHT_REPLAYS_FINISHED' "$DIR/full_suite.log" 2>/dev/null; then
  break
 fi
 if grep -q '^.*FAILED' "$DIR/full_suite.log" 2>/dev/null; then
  break
 fi
 sleep 30
done
