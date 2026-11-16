#!/usr/bin/env bash
# [ZAV 변경] Ubuntu/WSL 공통 센서 수신기와 RViz를 함께 실행합니다.
set -eo pipefail
rviz_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$rviz_dir/ros_env.sh"
source "$ZAV_WS_DIR/install/local_setup.bash"
exec 9>/tmp/pegasus-mid360-rviz.lock
if ! flock -n 9; then
    echo "Mid360 RViz is already running." >&2
    exit 1
fi
relay_pid=""
cleanup() {
    if [ -n "$relay_pid" ]; then
        kill "$relay_pid" 2>/dev/null || true
        for _wait in {1..30}; do
            kill -0 "$relay_pid" 2>/dev/null || break
            sleep 0.1
        done
        kill -KILL "$relay_pid" 2>/dev/null || true
        wait "$relay_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
# [ZAV 변경] 외부 Python ABI에서 Livox 메시지를 생성하므로 두 환경 모두 수신기가 필요합니다.
if [ "$ZAV_ROS_TRANSPORT" = cyclone_tcp ]; then
    export CYCLONEDDS_URI="file://$rviz_dir/cyclonedds_publisher.xml"
fi
python3 "$rviz_dir/../runtime/sensor_relay.py" client --host "${ZAV_SENSOR_HOST:-127.0.0.1}" --camera-port 17443 --imu-port 17444 --livox-port 17445 \
    > /tmp/mid360-tcp-wsl.log 2>&1 9>&- &
relay_pid=$!
echo "LiDAR: /drone0/mid360/pointcloud | frame: mid360"
echo "IMU: /drone0/mid360/imu | frame: mid360_imu"
echo "FAST-LIVO: /drone0/mid360/lidar | Livox CustomMsg"
echo "Relay log: /tmp/mid360-tcp-wsl.log"
echo "RViz: /cloud_registered + /aft_mapped_to_init + /path | fixed frame: camera_init"
# 수신기의 고정 포트를 viewer가 물려받지 않게 클라이언트 설정으로 복구합니다.
if [ "$ZAV_ROS_TRANSPORT" = cyclone_tcp ]; then
    export CYCLONEDDS_URI="file://$rviz_dir/cyclonedds_local.xml"
fi
bash "$rviz_dir/gui_exec.sh" rviz2 -d "$rviz_dir/mid360.rviz" 9>&-
