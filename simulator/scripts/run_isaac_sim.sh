#!/usr/bin/env bash
# 원본의 tmux + 별도 Isaac Python 구조를 유지합니다.
_config="${ZAV_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/zav/simulation.env}"
[ ! -f "$_config" ] || source "$_config"
SESSION_NAME=zav_sim
ISAAC_LOG=/tmp/zav_isaac_sim.log
ZAV_WS_DIR="${ZAV_WS_DIR:-$HOME/zenith_ws}"
SIM_SCRIPT="$ZAV_WS_DIR/simulator/zav_sim.py"
trap 'tmux kill-session -t "$SESSION_NAME" 2>/dev/null; exit 130' INT TERM

# [ZAV 변경] 환경 판별을 한 곳에서 수행하고 테스트/WSL 내 Linux 실행 시 명시적 override를 허용합니다.
_platform="${ZAV_SIM_PLATFORM:-auto}"
if [ "$_platform" = auto ]; then
    if grep -qi microsoft /proc/version; then _platform=wsl; else _platform=native; fi
fi
export PEGASUS_FASTLIVO_CONFIG_DIR="${PEGASUS_FASTLIVO_CONFIG_DIR:-$ZAV_WS_DIR/src/FAST-LIVO2/config}"
echo '[Isaac Sim] PX4 TCP 4560 준비 대기 (3초)...'
sleep 3
case "$_platform" in
    wsl)
        _bat="${ISAAC_WINDOWS_PYTHON_BAT:-}"
        if [ -z "$_bat" ]; then
            for candidate in /mnt/c/isaac-sim/python.bat /mnt/c/issac-sim/python.bat /mnt/c/Users/*/AppData/Local/ov/pkg/isaac*sim*/python.bat; do
                if [ -f "$candidate" ]; then _bat="$candidate"; break; fi
            done
        fi
        if [ ! -f "$_bat" ] || [ ! -f "$SIM_SCRIPT" ]; then
            echo 'Windows Isaac python.bat 또는 저장소의 simulator/zav_sim.py가 없습니다. ISAAC_WINDOWS_PYTHON_BAT를 설정하세요.' >&2
            exit 1
        fi
        # [ZAV 변경] 사용자 이름/Windows 프로젝트 경로 대신 설치된 작업 사본을 실행합니다.
        # WSLENV /p가 YAML 디렉토리를 Windows에서 읽을 수 있는 경로로 변환합니다.
        export WSLENV="${WSLENV:+$WSLENV:}PEGASUS_FASTLIVO_CONFIG_DIR/p:PEGASUS_SLAM_SCENE:PEGASUS_BIND_HOST:ROS_DOMAIN_ID:ZAV_DIAGNOSTIC_STATE"
        (cd /mnt/c && /mnt/c/Windows/System32/cmd.exe /c call "$(wslpath -m "$_bat")" "$(wslpath -m "$SIM_SCRIPT")") 2>&1 | tee "$ISAAC_LOG"
        EXIT_CODE=${PIPESTATUS[0]}
        ;;
    native)
        # [ZAV 변경] 일반 ROS venv/python3를 Isaac Python으로 잘못 선택하지 않습니다.
        _python="${ISAAC_PYTHON:-}"
        _root="${ISAAC_SIM_PATH:-}"
        if [ -z "$_python" ]; then
            for candidate in "${_root:+$_root/python.sh}" "$HOME/isaacsim/python.sh" "$HOME/isaac-sim/python.sh" "$HOME/isaac_env/bin/python" "$HOME/isaacsim_env/bin/python"; do
                if [ -n "$candidate" ] && [ -x "$candidate" ]; then _python="$candidate"; break; fi
            done
        fi
        if [ ! -x "$_python" ]; then
            echo 'Isaac Python을 찾지 못했습니다. ISAAC_PYTHON=/절대경로/python.sh 또는 ISAAC_SIM_PATH를 설정하세요.' >&2
            exit 1
        fi
        if [ -z "$_root" ]; then
            if [ "$(basename "$_python")" = python.sh ]; then
                _root="$(dirname "$_python")"
            else
                _root="$(env -u PYTHONPATH "$_python" -c 'import importlib.util,pathlib; s=importlib.util.find_spec("isaacsim"); print(pathlib.Path(s.origin).parent if s and s.origin else "")')"
            fi
        fi
        # [ZAV 변경] Humble apt Python 3.10과 Isaac 5.1 Python 3.11 ABI를 분리합니다.
        # 사용자 ROS/워크스페이스 라이브러리를 제거하고 Isaac 내부 Humble만 로드합니다.
        _clean_ld=""
        IFS=: read -ra _entries <<< "${LD_LIBRARY_PATH:-}"
        for entry in "${_entries[@]}"; do
            case "$entry" in /opt/ros/*|"$ZAV_WS_DIR"/install/*|*/pegasus-cyclonedds/*|'') continue ;; esac
            _clean_ld="${_clean_ld:+$_clean_ld:}$entry"
        done
        export LD_LIBRARY_PATH="$_root/exts/isaacsim.ros2.bridge/humble/lib${_clean_ld:+:$_clean_ld}"
        export ISAAC_PATH="$_root"
        env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u CYCLONEDDS_URI \
            -u FASTRTPS_DEFAULT_PROFILES_FILE -u FASTDDS_DEFAULT_PROFILES_FILE \
            ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
            "$_python" "$SIM_SCRIPT" 2>&1 | tee "$ISAAC_LOG"
        EXIT_CODE=${PIPESTATUS[0]}
        ;;
    *) echo "잘못된 ZAV_SIM_PLATFORM: $_platform" >&2; exit 1 ;;
esac
if [ "$EXIT_CODE" -ne 0 ]; then
    echo "Isaac Sim 종료 코드: $EXIT_CODE, 로그: $ISAAC_LOG" >&2
    if [ -t 0 ]; then read -r -n 1 -p '아무 키를 누르면 대시보드를 닫습니다.'; fi
fi
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
exit "$EXIT_CODE"
