#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source simulator/ros_setup.sh
exec python3 simulator/tools/runtime/record_odometry.py "$@"
