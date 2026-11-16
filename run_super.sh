#!/usr/bin/env bash
set -eo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/simulator/ros_setup.sh"
exec python3 "$ZAV_WS_DIR/simulator/tools/super/run_navigation.py" "$@"
