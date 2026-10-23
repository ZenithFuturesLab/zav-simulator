#!/usr/bin/env bash
# A pane's Ctrl+C closes the complete simulator dashboard.
set -eo pipefail
_config="${ZAV_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/zav/simulation.env}"
source "$_config"
trap 'tmux kill-session -t zav_sim 2>/dev/null || true' EXIT
trap 'exit 130' INT TERM
cd "$PX4_DIR"
PX4_SIM_HOSTNAME="${PX4_SIM_HOSTNAME:-localhost}" make px4_sitl isaac_zav
