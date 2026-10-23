# Ubuntu 22.04 설치

1. NVIDIA 드라이버와 Isaac Sim 5.1.0을 설치하고, NVIDIA의 기본 standalone 예제를 실행해 GPU 환경을 먼저 확인합니다. Isaac Sim 라이선스 동의는 NVIDIA 설치 절차에서 처리합니다.
2. README의 clone 명령을 실행합니다. 사용자 이름·설치 경로는 자유롭게 바꿀 수 있습니다. 소스와 빌드 결과를 다른 컴퓨터에서 복사하지 말고 해당 컴퓨터에서 빌드하세요.
3. `./scripts/bootstrap.sh --platform native --isaac-python /실제/경로/python.sh`를 실행합니다. pip 설치라면 Python 3.11 venv의 `bin/python`을 전달합니다.
4. 설치가 끝나면 새 터미널을 열고 README의 네 명령을 실행합니다.

설치 순서는 OS 의존성 및 ROS 2 Humble → 고정 커밋 다운로드 → 기체·센서·런처 배포 및 로컬 경로 저장 → Isaac Python 의존성 → Agent·PX4·ROS 패키지 빌드 → 읽기 전용 진단입니다. 다운로드나 빌드가 실패하면 실패 원인을 수정한 후 같은 bootstrap 명령을 재실행할 수 있습니다. 기존 외부 checkout의 커밋이 다르면 자동 reset하지 않고 중단합니다.

시스템 패키지는 apt, PX4 전용 Python 의존성은 `.cache/px4-python`, Agent와 그 라이브러리는 `.local`에 설치합니다. Agent를 `/usr/local`에 덮어쓰지 않습니다. ROS Humble은 시스템 Python 3.10, Isaac Sim은 내부 Python 3.11을 사용합니다. 실행기가 ROS의 Python 경로를 Isaac 프로세스에서 제거하므로 두 환경을 합치지 마세요. [NVIDIA ROS 설치 안내](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html).

ROS 저장소는 공식 `ros2-apt-source` 1.3.0 패키지로 등록합니다. Ubuntu 22.04를 막 설치한 경우 먼저 OS 업데이트를 마치고 재부팅한 다음 진행하세요. [ROS 2 Humble 공식 설치 문서](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst).

설치기는 `.bashrc`에 표시된 ZAV 블록을 추가하고, `~/.config/zav/simulation.env`와 홈 실행기를 등록합니다. 기존 파일은 최초 변경 전 `.before-zav-simulator` 사본으로 보관합니다. 홈 설정을 바꾸고 싶지 않으면 `--no-shell-integration`을 추가하고 매 터미널에서 `simulation/ros_setup.sh`를 직접 source하세요.

파일을 수정한 후 재배포·빌드:

```bash
python3 scripts/configure.py --platform native --isaac-python /actual/path/python.sh
./scripts/build.sh
python3 scripts/doctor.py
```

빌드 디렉토리를 삭제하는 PX4 clean/distclean은 저장 파라미터·비행 로그도 잃을 수 있습니다. 기존 설치를 정리하기 전 `src/PX4-Autopilot/build/px4_sitl_default/rootfs`를 별도 백업하세요. 설치기는 이 폴더를 자동 삭제하지 않습니다.

다른 로봇과 domain을 분리하려면 configure/bootstrap에 `--domain-id 30`처럼 지정합니다. 실행 전에 명시적으로 export한 `ROS_DOMAIN_ID`는 저장된 기본 domain보다 우선합니다. 네 터미널 모두 같은 domain을 사용해야 합니다.
