#!/usr/bin/env bash
export ZAV_ENV_FILE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/environment.sh"
source "$ZAV_ENV_FILE"
source "$PEGASUS_DIR/tools/rviz/ros_env.sh" || return 1
source "$ZAV_WS_DIR/install/local_setup.bash"
