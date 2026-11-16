#!/usr/bin/env bash
# Official FAST-LIO2 ROS2 estimator using the common mounted-IMU contract.
set -eo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/simulator/ros_setup.sh"
exec python3 "$ZAV_WS_DIR/simulator/tools/runtime/fastlivo_session.py" \
  --rviz-dir "$ZAV_WS_DIR/simulator/tools/rviz" --backend fastlio rviz:=False "$@"
