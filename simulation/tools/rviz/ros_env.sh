# [ZAV 변경] 공통 ROS 환경: Ubuntu는 Fast DDS, WSL은 검증된 Cyclone TCP를 사용합니다.
_zav_config="${ZAV_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/zav/simulation.env}"
[ ! -f "$_zav_config" ] || source "$_zav_config"
export PEGASUS_DIR="${PEGASUS_DIR:-$HOME/zenith_ws/src/PegasusSimulator}"
export ZAV_WS_DIR="${ZAV_WS_DIR:-$HOME/zenith_ws}"
export PX4_DIR="${PX4_DIR:-$HOME/zenith_ws/src/PX4-Autopilot}"
source /opt/ros/humble/setup.bash || return 1
_zav_rviz_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# 실행 위치와 DDS 선택은 독립적입니다 (WSL 안의 Linux Isaac도 명시적으로 선택 가능).
export ZAV_WINDOWS_HOST=0
if [ "${ZAV_SIM_PLATFORM:-auto}" = wsl ] || { [ "${ZAV_SIM_PLATFORM:-auto}" = auto ] && grep -qi microsoft /proc/version; }; then
    export ZAV_WINDOWS_HOST=1
fi
_zav_transport="${ZAV_ROS_TRANSPORT:-auto}"
if [ "$_zav_transport" = auto ]; then
    if [ "$ZAV_WINDOWS_HOST" = 1 ]; then _zav_transport=cyclone_tcp; else _zav_transport=fastdds; fi
fi
case "$_zav_transport" in
    cyclone_tcp)
        # 시스템 apt 설치와 기존 사용자 로컬 설치를 모두 허용합니다.
        _zav_dds_root="$HOME/.local/share/pegasus-cyclonedds/opt/ros/humble"
        if [ -f "$_zav_dds_root/lib/librmw_cyclonedds_cpp.so" ]; then
            export AMENT_PREFIX_PATH="$_zav_dds_root:${AMENT_PREFIX_PATH:-}"
            export CMAKE_PREFIX_PATH="$_zav_dds_root:${CMAKE_PREFIX_PATH:-}"
            export LD_LIBRARY_PATH="$_zav_dds_root/lib:$_zav_dds_root/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
        elif [ ! -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]; then
            echo 'Cyclone DDS가 필요합니다: sudo apt install ros-humble-rmw-cyclonedds-cpp' >&2
            return 1
        fi
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI="file://$_zav_rviz_dir/cyclonedds_local.xml"
        ;;
    fastdds)
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        unset CYCLONEDDS_URI
        ;;
    *) echo "잘못된 ZAV_ROS_TRANSPORT: $_zav_transport (auto/fastdds/cyclone_tcp)" >&2; return 1 ;;
esac
export ZAV_ROS_TRANSPORT="$_zav_transport"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
unset _zav_config _zav_rviz_dir _zav_transport _zav_dds_root
