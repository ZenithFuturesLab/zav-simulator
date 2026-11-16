#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export PYTHONNOUSERSITE=1
python3 scripts/check_release.py
python3 simulator/tests/check_source_layout.py
python3 simulator/tests/check_fastlivo_calibration_math.py
python3 simulator/tests/check_mounted_imu_math.py
python3 simulator/tests/check_px4_lockstep.py
python3 simulator/tests/check_fastlivo_shutdown.py
python3 simulator/tests/check_platforms.py
if [[ "${1:-}" == --ros ]]; then
    export ZAV_ENV_FILE="$ROOT/simulator/environment.sh"
    source "$ROOT/simulator/ros_setup.sh"
    export ROS_DOMAIN_ID=49
    timeout -k 5 60 python3 simulator/tests/check_sensor_publication.py
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp timeout -k 5 60 python3 simulator/tests/check_native_ros.py
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp timeout -k 5 60 python3 simulator/tests/check_native_ros.py
    timeout -k 5 60 python3 simulator/tests/check_bridge_launch.py
    timeout -k 5 100 python3 simulator/tests/check_dds_gateway.py
fi
