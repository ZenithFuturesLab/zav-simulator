#!/usr/bin/env bash
# Keep the original FAST-LIVO launch/calibration and give DDS a discoverable peer.
set -eo pipefail
rviz_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$rviz_dir/ros_env.sh"
source "$ZAV_WS_DIR/install/local_setup.bash"
# One foreground supervisor owns Ctrl+C/HUP, descendants and the launch lock.
exec python3 "$rviz_dir/../runtime/fastlivo_session.py" --rviz-dir "$rviz_dir" "$@"
