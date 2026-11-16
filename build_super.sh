#!/usr/bin/env bash
set -eo pipefail
_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_workspace/simulator/ros_setup.sh"
cd "$_workspace"
_deps="$_workspace/.cache/super-deps"
if [ ! -f "$_workspace/.local/lib/cmake/osqp/osqp-config.cmake" ]; then
  if [ ! -d "$_deps/osqp/.git" ]; then
    git clone --depth 1 --branch v0.6.3 --recurse-submodules --shallow-submodules https://github.com/osqp/osqp.git "$_deps/osqp"
  fi
  cmake -S "$_deps/osqp" -B "$_deps/osqp-build" -DCMAKE_INSTALL_PREFIX="$_workspace/.local" -DUNITTESTS=OFF -DPRINTING=OFF -DENABLE_MKL_PARDISO=OFF
  cmake --build "$_deps/osqp-build" -j "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
  cmake --install "$_deps/osqp-build"
fi
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
_lio_deps="$_workspace/.cache/super-lio-deps/extracted/usr/lib/x86_64-linux-gnu/cmake"
_extra=()
if [ -d "$_lio_deps/glog" ]; then
  _extra+=(-Dglog_DIR="$_lio_deps/glog" -Dgflags_DIR="$_lio_deps/gflags")
fi
exec colcon build --packages-up-to zav_super fast_lio super_lio fast_livo --executor sequential --cmake-args \
  -DROS_EDITION=ROS2 -DLIVOX_BUILD_DRIVER=OFF "${_extra[@]}" -Dosqp_DIR="$_workspace/.local/lib/cmake/osqp" -DCMAKE_BUILD_TYPE=Release -Dfastcdr_DIR="/opt/ros/$ROS_DISTRO/lib/cmake/fastcdr" "$@"
