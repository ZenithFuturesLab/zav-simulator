#!/usr/bin/env bash
# [ZAV 변경] 홈 명령은 설치된 공통 실행기로 위임합니다.
_config="${ZAV_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/zav/simulation.env}"
[ ! -f "$_config" ] || source "$_config"
exec bash "${PEGASUS_DIR:-$HOME/zenith_ws/src/PegasusSimulator}/tools/rviz/run_fastlivo.sh" rviz:=False camera:=True "$@"
