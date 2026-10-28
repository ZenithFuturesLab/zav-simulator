#!/usr/bin/env bash
export ZAV_ENV_FILE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/simulator/environment.sh"
source "$ZAV_ENV_FILE"
exec bash "$ZAV_WS_DIR/simulator/tools/rviz/run_fastlivo.sh" rviz:=False camera:=True "$@"
