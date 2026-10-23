#!/usr/bin/env bash
# Source before rosdep/colcon, including in CI and on a fresh checkout.
_zav_build_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
# Livox ships separate manifests; ament requires this exact file name.
cp "$_zav_build_root/src/livox_ros_driver2/package_ROS2.xml" "$_zav_build_root/src/livox_ros_driver2/package.xml" || return 1
# Ignore deleted workspaces and previously sourced DDS overlays.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash || return 1
unset _zav_build_root
