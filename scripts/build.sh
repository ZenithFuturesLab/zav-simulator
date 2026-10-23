#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
JOBS="${ZAV_BUILD_JOBS:-2}"
cd "$ROOT"
source "$ROOT/scripts/prepare_ros.sh"
python3 -m venv --system-site-packages .cache/px4-python
.cache/px4-python/bin/python -m pip install -r scripts/requirements-px4.txt
export PATH="$ROOT/.cache/px4-python/bin:$PATH"
# Source only Humble: do not pull binaries from a different workspace.
rosdep install --from-paths src/FAST-LIVO2 src/rpg_vikit src/sophus src/livox_ros_driver2 src/px4_msgs src/zav_control --ignore-src -r -y --rosdistro humble
cmake -S src/Micro-XRCE-DDS-Agent -B build/agent -DUAGENT_SUPERBUILD=ON -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$ROOT/.local"
cmake --build build/agent --parallel "$JOBS"
python3 scripts/check_agent_dependencies.py
cmake --install build/agent
# Keep the initial PX4 build before any user-generated parameter/log data.
make -C src/PX4-Autopilot -j"$JOBS" px4_sitl_default
# Agent's private Fast CDR 2.x must not replace Humble's Fast CDR 1.x.
# CMake also searches prefixes derived from PATH (.local/bin), not only CMAKE_PREFIX_PATH.
MAKEFLAGS="-j$JOBS" CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" colcon build --symlink-install --executor sequential --packages-up-to px4_msgs zav_control fast_livo --cmake-args -DROS_EDITION=ROS2 -DLIVOX_BUILD_DRIVER=OFF -DCMAKE_BUILD_TYPE=Release -Dfastcdr_DIR=/opt/ros/humble/lib/cmake/fastcdr -DCMAKE_IGNORE_PREFIX_PATH="$ROOT/.local"
