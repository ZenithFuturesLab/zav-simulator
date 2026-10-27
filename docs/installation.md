# ZAVSIM_Fix 설치·공동 개발 안내

개발 브랜치는 **`ZAVSIM_Fix`**입니다. 브랜치 이름은 대소문자를 구분합니다.
시뮬레이터와 Pegasus는 저장소 원본을 직접 실행하므로 Python 변경은 pull 후
재실행하면 적용됩니다. 최초 의존성 설치와 C++ 빌드는 각 컴퓨터에서 필요합니다.

## 1. 먼저 준비할 환경

| 환경 | 준비할 항목 |
| --- | --- |
| Ubuntu 단독 | Ubuntu 22.04 x86_64, NVIDIA 드라이버·GPU, Linux Isaac Sim **5.1.0** |
| Windows + WSL2 | Windows Isaac Sim **5.1.0**, NVIDIA Windows 드라이버, WSL2 Ubuntu 22.04, WSLg, mirrored networking |

Isaac Sim은 별도로 설치하고 기본 예제의 GPU 실행을 먼저 확인합니다.
WSL의 Windows 설정은 [Windows/WSL2 안내](windows-wsl2.md)의 사전 준비를 따릅니다.
설치에는 인터넷, sudo 권한과 충분한 디스크·메모리가 필요합니다.
저장소가 비공개이므로 친구 계정에도 저장소 접근 권한이 있어야 합니다.

## 2. 브랜치 받기

아래 명령은 Ubuntu 또는 WSL의 Ubuntu 터미널에서 실행합니다.
WSL에서는 소스를 Windows 드라이브 대신 Linux의 `~/zenith_ws`에 둡니다.

```bash
sudo apt-get update
sudo apt-get install -y git gh
gh auth login --hostname github.com --git-protocol https --web
gh auth setup-git

git clone --branch ZAVSIM_Fix --single-branch https://github.com/ZenithFuturesLab/zav-simulator.git ~/zenith_ws
cd ~/zenith_ws
git branch --show-current  # ZAVSIM_Fix 확인
```

이미 같은 경로에 작업 폴더가 있으면 다른 빈 경로에 clone하거나 아래의 기존 사용자
절차를 사용합니다. 다른 컴퓨터의 build/install/가상환경을 복사하지 않습니다.

## 3. 최초 설치·빌드

일반적인 Isaac 설치 경로라면 `./setup.sh`가 Ubuntu/WSL과 실행 파일을 탐색합니다.
설치 위치를 명확히 지정하려면 자신의 환경에 맞게 아래 **한 가지**를 실행합니다.

Ubuntu standalone Isaac 설치:

```bash
./setup.sh --platform native --isaac-python /실제/Isaac설치경로/python.sh
```

Ubuntu의 Isaac Python 3.11 가상환경 설치:

```bash
./setup.sh --platform native --isaac-python /실제/가상환경/bin/python
```

Windows Isaac + WSL2 구성:

```bash
./setup.sh --platform wsl --isaac-windows-python /mnt/c/isaac-sim/python.bat
```

경로는 실제 설치 위치로 바꾸고 공백이 있으면 따옴표로 감쌉니다.
일반 시스템 Python을 Isaac Python 대신 지정하지 않습니다.

설치기는 순서대로 OS/ROS 의존성 설치 → 고정 버전 외부 소스 다운로드 →
컴퓨터별 설정·PX4 airframe 등록 → Isaac Python 패키지 설치 →
Agent/PX4/ROS 빌드 → 설치 진단을 실행합니다.
Pegasus 코드를 다른 설치 폴더에 복사하거나 별도로 설치하지 않습니다.

```bash
./setup.sh --dry-run        # 실행할 설치 단계 확인, 설치하지 않음
./setup.sh --help           # 경로·domain 등 옵션
```

메모리가 부족하면 `ZAV_BUILD_JOBS=1 ./setup.sh ...`처럼 빌드 작업 수를 낮춥니다.
설치가 실패했다면 오류를 해결하고 같은 명령을 다시 실행할 수 있습니다.

## 4. 실행·확인

설치 후 새 터미널을 열고 각 터미널에서 저장소로 이동합니다.
셸 자동 등록을 사용하지 않았다면 먼저 `source simulator/ros_setup.sh`를 실행합니다.

| 터미널 | 명령 | 역할 |
| --- | --- | --- |
| 1 | `./run_zav_sim.sh` | Isaac + PX4 + DDS Agent |
| 2 | `./run_fastlivo.sh` | 기본 LiDAR+IMU 추정 + PX4 외부 위치 브리지 |
| 3 | `ros2 run zav_control flight_manager` | 비행 조작 |
| 4 | `./run_mid360_rviz.sh` | RViz 센서·추정 결과 표시 |

카메라 영상도 FAST-LIVO 추정에 사용하려면 터미널 2에서
`./run_fastlivo.sh camera:=True`로 실행합니다. 기본 실행은 `camera:=False`입니다.

```bash
python3 scripts/doctor.py
./scripts/verify.sh
source simulator/ros_setup.sh
python3 scripts/observe_runtime.py --duration 30
```

`doctor.py`는 설치·실행 파일·메시지 ABI를, `verify.sh`는 소스·수학·실행기 회귀를
검사합니다. `observe_runtime.py`는 실행 중인 시스템에서 센서·LIO·EV·PX4 위치를
수신 확인합니다. 비행 조작과 GPU 검증은 각 장비에서 추가로 확인해야 합니다.
종료는 FAST-LIVO 터미널의 Ctrl+C와 시뮬레이터 대시보드의 Ctrl+C를 사용합니다.

알려진 DDS 재연결 문제, 격리 ROS 검사 실패, FAST-LIVO 종료 오류 및 실제 비행
검증 범위는 [검증 기록](verification.md)에 명시했습니다. `verify.sh --ros` 전체가
통과한 상태로 안내하지 않습니다. 새 컴퓨터 전체 설치는 해당 컴퓨터에서 확인해야 합니다.

## 5. 이후 코드를 주고받기

수정한 사람:

```bash
git status
git add <수정한-파일>
git commit -m "변경 내용"
git push
```

받는 사람은 실행 중인 시뮬레이터와 ROS 노드를 종료하고:

```bash
cd ~/zenith_ws
git pull --ff-only
```

| 받은 변경 | 추가 작업 |
| --- | --- |
| `simulator/` Python·USD·설정, 기존 ROS Python 모듈 | 관련 프로세스 재실행 |
| FAST-LIVO 등 C++, ROS 메시지·패키지 설치 정의·새 Python 모듈 | `./scripts/build.sh` 후 재실행 |
| `simulator/px4/` airframe | 아래의 configure 후 `./scripts/build.sh` |
| 설치 스크립트·requirements·의존성 lock | 자신의 최초 `./setup.sh ...` 명령 재실행 |

`git pull --ff-only`가 충돌이나 분기 때문에 거절되면 자신의 변경을 보존한 뒤
팀원과 병합합니다. 서로 같은 파일을 수정하면 Git 병합이 필요할 수 있습니다.

기존 `simulation/` 구조를 사용하던 사람은 처음 이 브랜치를 받은 뒤 **한 번**
자신의 경로와 기존 domain 값을 지정하여 설정을 갱신합니다. 예:

```bash
# Windows + WSL2
python3 scripts/configure.py --platform wsl --isaac-windows-python /mnt/c/isaac-sim/python.bat
# Ubuntu (자신의 실제 Isaac 경로 사용)
python3 scripts/configure.py --platform native --isaac-python /실제/Isaac설치경로/python.sh
```

두 명령 중 환경에 맞는 하나를 사용하고 새 터미널을 엽니다.
기존에 별도 ROS domain을 썼다면 `--domain-id 기존값`을 추가합니다.
새 clone이거나 필요한 의존성이 없으면 configure 대신 최초 setup 절차를 실행합니다.
이전 `src/PegasusSimulator` checkout은 더 이상 실행에 사용하지 않습니다.

## 6. 설치에 필요한 파일은 어디 있나

| 파일·폴더 | 역할 |
| --- | --- |
| `setup.sh` | 최초 설치 진입점 |
| `scripts/bootstrap.sh` | 설치 전체 순서 |
| `scripts/install_system.sh` | Ubuntu/ROS apt 의존성 |
| `dependencies.lock.json`, `scripts/fetch_dependencies.py` | PX4·Agent 다운로드 URL·고정 커밋 |
| `agent-dependencies.lock.json` | Agent 내부 DDS 의존성 버전 |
| `scripts/requirements-isaac.txt` | Linux·Windows 공통 Isaac Python 패키지 버전 |
| `scripts/requirements-px4.txt` | PX4 빌드 Python 의존성 |
| `scripts/configure.py` | Isaac 경로·플랫폼·domain, PX4 설정 |
| `scripts/build.sh`, `scripts/prepare_ros.sh` | Agent·PX4·ROS 빌드 |
| `simulator/` | 직접 실행하는 센서·기체·Pegasus 소스·USD·설정 |
| `src/`의 ROS 소스, 각 `package.xml`·CMake/setup 파일 | FAST-LIVO·메시지·제어 패키지 및 빌드 정의 |
| `simulator/pegasus/LICENSE`, `UPSTREAM.json`, `THIRD_PARTY_NOTICES.md` | 포함한 외부 소스 출처·라이선스 |

Git에는 위 소스와 설치 정의를 포함합니다. `.zav.env`는 각 컴퓨터에서 생성합니다.
Isaac 설치, `.venv`, `.cache`, `.local`, `build`, `install`, `log`, 비행 기록과
다운로드한 PX4·Agent checkout은 Git에 올리지 않으며 설치기가 준비합니다.
