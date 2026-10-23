#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONNOUSERSITE=1
python3 scripts/check_release.py
python3 simulation/tests/check_deployment.py
python3 simulation/tests/check_fastlivo_calibration_math.py
python3 simulation/tests/check_mounted_imu_math.py
python3 simulation/tests/check_px4_lockstep.py
python3 simulation/tests/check_platforms.py
if [[ "${1:-}" == --ros ]]; then
    export ZAV_ENV_FILE="$ROOT/simulation/environment.sh"
    source "$ROOT/simulation/ros_setup.sh"
    export ROS_DOMAIN_ID=49
    python3 simulation/tests/check_sensor_publication.py
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp python3 simulation/tests/check_native_ros.py
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 simulation/tests/check_native_ros.py
    python3 simulation/tests/check_bridge_launch.py
    python3 simulation/tests/check_dds_gateway.py
fi
