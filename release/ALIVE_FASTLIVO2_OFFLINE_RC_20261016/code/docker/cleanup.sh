#!/usr/bin/env bash
set -euo pipefail

# Only names owned by this project are targeted.
docker rm -f fast_livo2_dev 2>/dev/null || true
docker rmi fast_livo2:dev 2>/dev/null || true
echo "Removed project-specific fast_livo2_dev and fast_livo2:dev when present."
