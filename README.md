# ZAV Simulator

Isaac Sim **5.1.0** + Pegasus + PX4 **v1.14.3** 기반 GPS 없는 드론 시뮬레이터입니다. MID-360 형태의 LiDAR, 카메라와 장착 IMU를 FAST-LIVO2에 입력하고, 추정 odometry를 ROS 2 노드를 통해 PX4 EKF2로 전달합니다.

이 저장소의 `src/`에 ROS 패키지를 포함합니다. Pegasus 소스는 `simulator/pegasus/`에서 직접 관리합니다. PX4·Micro XRCE-DDS Agent만 설치 스크립트가 고정된 Git 커밋으로 내려받습니다. Isaac Sim 자체, 가상환경, 빌드 결과, 개인 BSON·로그는 포함하지 않습니다.

## 지원 환경

| 환경 | 실행 구성 | 확인 범위 |
| --- | --- | --- |
| Ubuntu 22.04 x86_64 + NVIDIA GPU | Linux Isaac Sim + ROS 2 Humble | 배포본에서 GUI·센서·LIO→EKF 위치 확인. 상세 검증은 [검증 기록](docs/verification.md) 참조 |
| Windows 11 22H2 이상 + WSL2 Ubuntu 22.04 | Windows Isaac Sim + WSL ROS 2/PX4, mirrored networking | Windows GPU 실행, 센서·FAST-LIVO·EV, 이륙·위치 이동·터미널 조종·착륙 확인. [검증 기록](docs/verification.md) 참조 |

Isaac Sim의 [GPU·드라이버 요구사항](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)을 충족해야 합니다. 설치 스크립트가 ROS 2와 컴파일 의존성을 설치하므로 인터넷·sudo 권한·충분한 여유 디스크가 필요합니다. 다른 Ubuntu 버전, ARM, Windows 10, WSL 기본 NAT를 동일 지원 환경으로 가정하지 않습니다.

## 처음 설치하기

개발 브랜치는 **`ZAVSIM_Fix`**입니다(대소문자 구분). 친구에게 전달할 전체 절차와
필요 파일 목록은 **[설치·공동 개발 안내](docs/installation.md)**를 참고하세요.

GitHub 조직의 비공개 저장소 접근 권한이 있는 계정으로 Git 인증을 먼저 설정합니다. 아래 저장소를 clone합니다.

```bash
sudo apt-get update
sudo apt-get install -y git software-properties-common
sudo add-apt-repository -y universe
sudo apt-get update
sudo apt-get install -y gh
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git
git clone --branch ZAVSIM_Fix --single-branch https://github.com/ZenithFuturesLab/zav-simulator.git ~/zenith_ws
cd ~/zenith_ws
```

`gh auth login`이 표시하는 일회용 코드와 주소를 사용해 브라우저에서 GitHub 로그인을 완료합니다. `ZenithFuturesLab` 접근 권한이 있는 계정이어야 합니다.

이미 `~/zenith_ws`가 있으면 다른 빈 경로로 clone하세요. 기존 폴더에 덮어쓰거나 기존 작업을 삭제하지 마세요.

**Ubuntu:** Isaac Sim에서 제공하는 `python.sh` 또는 Isaac Sim 5.1.0이 설치된 Python 3.11 가상환경의 Python을 지정합니다. 일반 `/usr/bin/python3`은 사용하지 않습니다.

```bash
./setup.sh --platform native --isaac-python /absolute/path/to/isaac-sim/python.sh
```

pip 방식 설치라면 다음처럼 지정합니다.

```bash
./setup.sh --platform native --isaac-python /absolute/path/to/isaac_env/bin/python
```

**Windows + WSL2:** 먼저 [WSL2 설치 안내](docs/windows-wsl2.md)에 따라 mirrored networking과 Windows Isaac Sim 실행을 확인하고, WSL 터미널에서 실행합니다.

```bash
./setup.sh --platform wsl --isaac-windows-python /mnt/c/isaac-sim/python.bat
```

실행 전 설치 단계를 확인하려면 `./scripts/bootstrap.sh --dry-run`을 사용합니다. 전체 옵션은 `--help`로 확인합니다. 작업 수 기본값은 2이며 `ZAV_BUILD_JOBS=4 ./scripts/bootstrap.sh ...`처럼 조절할 수 있습니다.

## 실행하기

설치가 끝나면 새 터미널 네 개에서 아래 명령을 각각 실행합니다. `./run_*.sh`는 clone한 저장소 루트에서 실행합니다. 설치 시 홈 디렉토리에도 호환 실행기가 만들어집니다.

```bash
# 터미널 1: Isaac Sim + PX4 + DDS Agent
zav_sim
# 터미널 2: FAST-LIVO2 + LIO → PX4 ROS 2 bridge
./run_fastlivo.sh
# 터미널 3: 비행 관리자
ros2 run zav_control flight_manager
# 터미널 4: 센서 RViz
./run_mid360_rviz.sh
```

새 터미널 환경 등록 없이 실행하려면 각 터미널에서 `source /path/to/zenith_ws/simulator/ros_setup.sh`를 먼저 실행하고, 시뮬레이터는 저장소 루트의 `./run_zav_sim.sh`로 실행합니다. 브리지 Python을 따로 실행할 필요가 없습니다.

첫 실행은 Isaac shader 캐시와 확장 로딩 때문에 오래 걸릴 수 있습니다. 비행 명령 전에 LiDAR·IMU·카메라와 EV odometry 수신을 [검증 절차](docs/verification.md)로 확인하세요. 시뮬레이터 대시보드에서 Ctrl+C로 종료합니다. 종료기는 동일 사용자 PX4·DDS Agent 및 Windows `kit.exe`도 정리하므로 다른 시뮬레이터와 동시에 실행하지 마세요.

FAST-LIVO는 실행한 터미널에서 Ctrl+C로 종료합니다. `[FAST-LIVO] All owned processes stopped`가 출력된 뒤 재실행하세요. 실행기가 만든 mapper·LIO bridge·relay를 정리한 후 재실행 잠금을 해제합니다. RViz가 먼저 실행해 소유한 공유 relay는 유지합니다. 잠금 파일을 직접 삭제하지 마세요. 이전 mapper가 남아 있으면 새 실행을 거부하고 PID를 표시합니다.

## LIO 이상 감시와 착륙

`./run_fastlivo.sh`는 짧은 측정 간격에서 위치·자세가 갑자기 튀는 운동 연속성만 감시합니다.
정상 비행에서는 기압계 융합을 끄고, 점프 확정 시 odometry와 외부 setpoint를 차단한 뒤
기압계 융합을 켜고 실제 융합·수직 상태를 확인해 PX4에 착륙을 요청합니다.
수신 지연·긴 측정 간격·LiDAR 보정 신선도·EKF2 융합 상태는 착륙 발동 기준에서 제거했습니다.
기본 임계값, 긴 간격의 비교 생략, 위치/자세 점프 실험 및 높이 추정 의존성은
[LIO safety 실험 안내](docs/lio-safety.md)를 확인하세요.

## FAST-LIVO odometry CSV 저장

시뮬레이터와 `./run_fastlivo.sh`를 실행한 상태에서 별도 터미널로 실행합니다.

```bash
./record_odometry.sh
# 저장 경로 지정
./record_odometry.sh --output log/odometry/flight.csv
# IMU 전파 odometry를 저장할 경우
./record_odometry.sh --topic /LIVO2/imu_propagate
```

기본 토픽은 `/aft_mapped_to_init`, 기본 파일은 `log/odometry/fastlivo_날짜_시간.csv`입니다.
Ctrl+C로 저장을 종료합니다. 기존 파일은 덮어쓰지 않으며 수신한 행마다 flush합니다.
CSV에는 메시지의 측정 시각(`stamp_sec`, `stamp_nanosec`, `timestamp_s`), 프레임,
위치(m), 자세 쿼터니언(x/y/z/w), 선속도(m/s), 각속도(rad/s)를 기록합니다.
좌표 변환 없이 원본 값을 저장하므로 pose는 `frame_id`, twist는 `child_frame_id` 기준입니다.
시뮬레이션의 측정 시각은 실제 날짜/시간과 다릅니다. 각속도는 발행된 값 그대로이며,
FAST-LIVO에서 채우지 않은 필드는 0일 수 있습니다. 기록기를 실행한 뒤 수신한 메시지만 저장합니다.

CSV를 저장하는 동안 별도 터미널에서 실시간 그래프를 실행할 수 있습니다. ROS 환경 설정은 필요 없습니다.

```bash
python3 simulator/tools/runtime/plot_odometry.py
# 파일을 직접 지정하거나 갱신 간격(ms)과 표시할 최대 샘플 수 조정
python3 simulator/tools/runtime/plot_odometry.py log/odometry/flight.csv --interval 200 --max-points 10000
```

파일을 생략하면 가장 최근 `fastlivo_*.csv`를 선택한 뒤 해당 파일을 계속 읽습니다.
위치 XYZ, 자세 roll/pitch/yaw(도), 선속도 XYZ, XYZ 3차원 궤적을 표시합니다.
3차원 궤적은 마우스로 드래그해 회전할 수 있으며 세 축의 거리 비율을 동일하게 유지합니다.
시간축은 첫 메시지 기준 측정 경과 시간이며, 자세는 쿼터니언에서 ZYX Euler로 변환합니다.
matplotlib이 없으면 `sudo apt install python3-matplotlib`로 설치합니다. 그래프 창을 닫으면 종료됩니다.

## 개발할 때

```text
simulator/
  zav_sim.py                 # 물리 주기·센서 설정·장면·메인 루프
  slam_test_scene.py         # SLAM 검증 장면
  pegasus/simulator/logic/   # 기체·센서·PX4/ROS 백엔드 (실제로 import하는 코드)
  pegasus/simulator/assets/  # 기체 USD·LiDAR USD
  config/                   # 공통 Pegasus 설정
  tools/                    # 센서 relay·FAST-LIVO 실행·RViz
  scripts/                  # Ubuntu/WSL Isaac·PX4 실행
src/                        # FAST-LIVO2 등 ROS 패키지
scripts/                    # 최초 설치·빌드·검증
.zav.env                    # 이 컴퓨터의 Isaac 경로·플랫폼 (Git 제외)
```

**Python 수정은 저장 후 재실행하면 적용됩니다.** 실행기가 저장소의
`simulator/zav_sim.py`를 직접 실행하고 같은 저장소의 Pegasus 코드를 import합니다.
설치 폴더로 복사하거나 deploy 명령을 실행하지 않습니다. Windows Isaac도 WSL의
동일 파일을 UNC 경로로 읽습니다. 기존 `src/PegasusSimulator`는 더 이상 사용하지 않습니다.

친구와 공유할 때는 수정한 원본을 commit/push하고 친구는 `git pull` 후 재실행합니다.
C++/ROS 메시지를 바꿨을 때만 `./scripts/build.sh`로 빌드합니다.
의존성 버전을 바꿨다면 최초 설치 명령을 다시 실행합니다.

최초 설치는 `./setup.sh`로 시작할 수 있습니다. Ubuntu/WSL을 자동 감지하고
일반적인 Isaac 경로를 찾습니다. 다른 위치에 설치했다면 위의 `--isaac-python` 또는
`--isaac-windows-python` 옵션을 지정하세요. 기존 checkout에서 업데이트한 경우에는
`python3 scripts/configure.py`를 한 번 실행해 이전 셸 등록 경로를 갱신하세요.

## 설정과 구조

- [Ubuntu 설치 상세](docs/ubuntu22.04.md), [Windows/WSL2](docs/windows-wsl2.md)
- [파라미터 적용 순서와 현재 프로파일](docs/parameters.md)
- [검증·문제 해결](docs/verification.md), [GitHub 업로드](docs/publishing.md)
- `simulator/`: 실행 코드·센서·기체 USD·Pegasus 소스·PX4 airframe
- `src/`: FAST-LIVO2, vikit, Sophus, Livox 메시지, PX4 메시지, `zav_control`
- `dependencies.lock.json`: 외부 소스 고정 버전
- `.zav.env`: 설치기가 만드는 컴퓨터별 경로. Git에 올리지 않음

`simulator/current_ekf_mpc.params`는 현재 재현 프로파일을 사람이 읽기 쉽게 정리한 파일입니다. 실제 기본값은 `simulator/px4/10020_isaac_zav`에서 설정합니다. 이 프로파일은 SITL 전용입니다.

각 외부 코드의 라이선스는 해당 파일과 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)를 따릅니다.
