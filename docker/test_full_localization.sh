#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FULL=1 exec "$SCRIPT_DIR/test_global_localization.sh" "$@"
