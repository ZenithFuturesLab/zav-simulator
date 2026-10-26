# ZAV Simulator

Isaac Sim **5.1.0** + Pegasus + PX4 **v1.14.3** 기반 GPS 없는 드론 시뮬레이터입니다. MID-360 형태의 LiDAR, 카메라와 장착 IMU를 FAST-LIVO2에 입력하고, 추정 odometry를 ROS 2 노드를 통해 PX4 EKF2로 전달합니다.

이 저장소의 `src/`에 ROS 패키지를 포함합니다. PX4·Pegasus·Micro XRCE-DDS Agent는 설치 스크립트가 고정된 Git 커밋으로 내려받습니다. Isaac Sim 자체, 가상환경, 빌드 결과, 개인 BSON·로그는 포함하지 않습니다.

## 지원 환경

| 환경 | 실행 구성 | 확인 범위 |
| --- | --- | --- |
| Ubuntu 22.04 x86_64 + NVIDIA GPU | Linux Isaac Sim + ROS 2 Humble | 배포본에서 GUI·센서·LIO→EKF 위치 확인. 상세 검증은 [검증 기록](docs/verification.md) 참조 |
| Windows 11 22H2 이상 + WSL2 Ubuntu 22.04 | Windows Isaac Sim + WSL ROS 2/PX4, mirrored networking | Windows GPU 실행, 센서·FAST-LIVO·EV, 이륙·위치 이동·터미널 조종·착륙 확인. [검증 기록](docs/verification.md) 참조 |

Isaac Sim의 [GPU·드라이버 요구사항](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)을 충족해야 합니다. 설치 스크립트가 ROS 2와 컴파일 의존성을 설치하므로 인터넷·sudo 권한·충분한 여유 디스크가 필요합니다. 다른 Ubuntu 버전, ARM, Windows 10, WSL 기본 NAT를 동일 지원 환경으로 가정하지 않습니다.

## 처음 설치하기

GitHub 조직의 비공개 저장소 접근 권한이 있는 계정으로 Git 인증을 먼저 설정합니다. 아래 저장소를 clone합니다.

```bash
sudo apt-get update
sudo apt-get install -y git software-properties-common
sudo add-apt-repository -y universe
sudo apt-get update
sudo apt-get install -y gh
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git
git clone https://github.com/ZenithFuturesLab/zav-simulator.git ~/zenith_ws
cd ~/zenith_ws
```

`gh auth login`이 표시하는 일회용 코드와 주소를 사용해 브라우저에서 GitHub 로그인을 완료합니다. `ZenithFuturesLab` 접근 권한이 있는 계정이어야 합니다.

이미 `~/zenith_ws`가 있으면 다른 빈 경로로 clone하세요. 기존 폴더에 덮어쓰거나 기존 작업을 삭제하지 마세요.

**Ubuntu:** Isaac Sim에서 제공하는 `python.sh` 또는 Isaac Sim 5.1.0이 설치된 Python 3.11 가상환경의 Python을 지정합니다. 일반 `/usr/bin/python3`은 사용하지 않습니다.

```bash
./scripts/bootstrap.sh --platform native --isaac-python /absolute/path/to/isaac-sim/python.sh
```

pip 방식 설치라면 다음처럼 지정합니다.

```bash
./scripts/bootstrap.sh --platform native --isaac-python /absolute/path/to/isaac_env/bin/python
```

**Windows + WSL2:** 먼저 [WSL2 설치 안내](docs/windows-wsl2.md)에 따라 mirrored networking과 Windows Isaac Sim 실행을 확인하고, WSL 터미널에서 실행합니다.

```bash
./scripts/bootstrap.sh --platform wsl --isaac-windows-python /mnt/c/isaac-sim/python.bat
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

새 터미널 환경 등록 없이 실행하려면 각 터미널에서 `source /path/to/zenith_ws/simulation/ros_setup.sh`를 먼저 실행하고, 시뮬레이터는 저장소 루트의 `./run_zav_sim.sh`로 실행합니다. 브리지 Python을 따로 실행할 필요가 없습니다.

첫 실행은 Isaac shader 캐시와 확장 로딩 때문에 오래 걸릴 수 있습니다. 비행 명령 전에 LiDAR·IMU·카메라와 EV odometry 수신을 [검증 절차](docs/verification.md)로 확인하세요. 시뮬레이터 대시보드에서 Ctrl+C로 종료합니다. 종료기는 동일 사용자 PX4·DDS Agent 및 Windows `kit.exe`도 정리하므로 다른 시뮬레이터와 동시에 실행하지 마세요.

## 설정과 구조

- [Ubuntu 설치 상세](docs/ubuntu22.04.md), [Windows/WSL2](docs/windows-wsl2.md)
- [파라미터 적용 순서와 현재 프로파일](docs/parameters.md)
- [검증·문제 해결](docs/verification.md), [GitHub 업로드](docs/publishing.md)
- `simulation/`: 센서·기체 USD·Pegasus 수정본·PX4 airframe·실행기 원본
- `src/`: FAST-LIVO2, vikit, Sophus, Livox 메시지, PX4 메시지, `zav_control`
- `dependencies.lock.json`: 외부 소스 고정 버전
- `.zav.env`: 설치기가 만드는 컴퓨터별 경로. Git에 올리지 않음

`simulation/current_ekf_mpc.params`는 현재 재현 프로파일을 사람이 읽기 쉽게 정리한 파일입니다. 실제 기본값은 `simulation/px4/10020_isaac_zav`에서 설정합니다. 이 프로파일은 SITL 전용입니다.

각 외부 코드의 라이선스는 해당 파일과 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)를 따릅니다.

test