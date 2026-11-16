#!/usr/bin/env bash
set -eo pipefail
_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_workspace/simulator/ros_setup.sh"
exec python3 "$_workspace/simulator/tools/super/show_flight.py" "$@"
