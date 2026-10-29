#!/usr/bin/env bash
# Dependencies on Ubuntu 22.04: libgoogle-glog-dev libtbb-dev (plus existing ROS/PCL).
set -eo pipefail
export ZAV_ENV_FILE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/simulator/environment.sh"
source "$ZAV_ENV_FILE"
source "$ZAV_WS_DIR/simulator/ros_setup.sh"
cd "$ZAV_WS_DIR"
# Keep Humble's Fast-CDR 1.x separate from the PX4 build's local Fast-CDR 2.x.
_cmake_args=(-DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DLIVOX_BUILD_DRIVER=OFF -Dfastcdr_DIR="/opt/ros/$ROS_DISTRO/lib/cmake/fastcdr")
_deps="$ZAV_WS_DIR/.cache/super-lio-deps/extracted/usr/lib/x86_64-linux-gnu/cmake"
if [ -d "$_deps/glog" ]; then
    _cmake_args+=(-Dglog_DIR="$_deps/glog" -Dgflags_DIR="$_deps/gflags")
fi
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
exec colcon build --packages-up-to super_lio --executor sequential \
    --cmake-args "${_cmake_args[@]}" "$@"
