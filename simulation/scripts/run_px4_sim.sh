#!/usr/bin/env bash
# A pane's Ctrl+C closes the complete simulator dashboard.
set -eo pipefail
_config="${ZAV_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/zav/simulation.env}"
source "$_config"
trap 'tmux kill-session -t zav_sim 2>/dev/null || true' EXIT
trap 'exit 130' INT TERM
cd "$PX4_DIR"
# On WSL a connect() issued before the Windows listener exists can stay in
# SYN-SENT for minutes. Inspect the listener without consuming MAVLink's socket.
if [ "${ZAV_SIM_PLATFORM:-}" = wsl ]; then
    echo '[PX4] Waiting for Windows Isaac MAVLink listener on TCP 4560...'
    _ready=0
    for _attempt in {1..180}; do
        if /mnt/c/Windows/System32/cmd.exe /c 'netstat -ano -p tcp' 2>/dev/null | grep -E ':4560[[:space:]].*LISTENING' >/dev/null; then
            _ready=1
            break
        fi
        sleep 1
    done
    [ "$_ready" = 1 ] || { echo 'Isaac MAVLink listener did not start within 180 seconds.' >&2; exit 1; }
fi
PX4_SIM_HOSTNAME="${PX4_SIM_HOSTNAME:-localhost}" make px4_sitl isaac_zav
