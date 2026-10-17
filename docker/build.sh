#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# The benchmark runner launches fast_livo2_failsafe:dev.  Keep the historical
# fast_livo2:dev tag too so older manual commands remain compatible, but never
# leave the benchmark image stale after a C++ change.
docker build --file "$PACKAGE_DIR/docker/Dockerfile" \
  --tag fast_livo2_failsafe:dev \
  --tag fast_livo2:dev \
  "$PACKAGE_DIR"
