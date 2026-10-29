#!/usr/bin/env bash
# Super-LIO: same Mid360 inputs, PX4 bridge and LIO safety as run_fastlivo.sh.
set -eo pipefail
export ZAV_ENV_FILE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/simulator/environment.sh"
source "$ZAV_ENV_FILE"
source "$ZAV_WS_DIR/simulator/tools/rviz/ros_env.sh"
source "$ZAV_WS_DIR/install/local_setup.bash"
exec python3 "$ZAV_WS_DIR/simulator/tools/runtime/fastlivo_session.py" \
    --rviz-dir "$ZAV_WS_DIR/simulator/tools/rviz" --backend super_lio rviz:=False "$@"
