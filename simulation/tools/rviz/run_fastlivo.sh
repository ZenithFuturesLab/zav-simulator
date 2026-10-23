#!/usr/bin/env bash
# Keep the original FAST-LIVO launch/calibration and give DDS a discoverable peer.
set -eo pipefail
rviz_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$rviz_dir/ros_env.sh"
source "$ZAV_WS_DIR/install/local_setup.bash"
# [ZAV 변경] 같은 입력에 mapper 두 개가 odom/TF를 동시에 발행하지 않게 중복 실행을 막습니다.
exec 9>"/tmp/zav-fastlivo-$UID.lock"
if ! flock -n 9; then
    echo 'FAST-LIVO가 이미 실행 중입니다. 기존 실행을 종료한 후 다시 시작하세요.' >&2
    exit 1
fi
# The CustomMsg relay is required even when RViz is not running.
python3 "$rviz_dir/../runtime/sensor_relay.py" client \
    --host "${ZAV_SENSOR_HOST:-127.0.0.1}" --camera-port 17443 --imu-port 17444 --livox-port 17445 \
    > /tmp/zav-fastlivo-relay.log 2>&1 9>&- &
relay_pid=$!
mapper_pid=""
cleanup() {
    if [ -n "$mapper_pid" ]; then
        # Launch and its ROS nodes share a dedicated process group.
        kill -TERM -- "-$mapper_pid" 2>/dev/null || true
        wait "$mapper_pid" 2>/dev/null || true
    fi
    kill -TERM "$relay_pid" 2>/dev/null || true
    wait "$relay_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
setsid ros2 launch "$rviz_dir/fastlivo_sim.launch.py" "$@" &
mapper_pid=$!
wait "$mapper_pid"
