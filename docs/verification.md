# 검증과 문제 해결

## 저장소 원본 직접 실행 구조 검증 (2026-10-26)

`simulation/`을 `simulator/`로 정리하고 Pegasus 기반 소스를 함께 포함했습니다.
`deploy_sim.py`와 overlay 복사를 제거했습니다. 실행기는 `simulator/zav_sim.py`를
직접 실행하며, Windows Isaac도 WSL 저장소의 같은 파일을 UNC 경로로 읽습니다.
이번 실행은 기존 ROS/PX4 빌드 결과를 사용했습니다. 새 clone의 전체 재설치·재빌드는
실행하지 않았습니다. 기존 Pegasus checkout은 `.cache/refactor-backup/PegasusSimulator-legacy`로 옮긴
상태에서 아래 검사를 진행했습니다. 기존 소스 사본도 같은 백업 디렉토리에 보존했습니다.

센서·통신 백엔드·기체·장면·ROS 도구 20개 구성요소는 변경 전 파일과 바이트 단위로
같음을 확인했습니다. 물리/IMU 160Hz, 렌더링/RGB 20Hz 설정을 유지했습니다.
기존 문서에 남아 있는 200Hz 기록은 이전 실행의 설정입니다.

| 검사 | 이번 결과 |
| --- | --- |
| 소스 구문·셸·XML·설치 dry-run, `scripts/verify.sh` | 통과 |
| 외부 Pegasus 없는 공백 포함 경로의 소스·설정·USD 확인 | 통과 |
| 캘리브레이션·IMU·lockstep·종료/재시작 회귀 검사 | 통과 |
| native/WSL 환경 선택, native Isaac Python ABI 분리 | 테스트용 실행기로 통과 |
| 설치 진단 `scripts/doctor.py` | 통과 |
| Windows Isaac + WSL 실제 GUI·RViz·센서→FAST-LIVO→EV→PX4 EKF | 통과 |
| 2m 이륙·호버 | 고도 1.95m 이륙 완료, 이후 약 2m 유지 |
| 북쪽/동쪽 1m 이동 | NED 약 `(0.920, 0.894, -1.985)m` |
| 원점 복귀 | NED 약 `(0.099, 0.110, -1.993)m` |
| 착륙 | PX4 `Landing detected`, `Disarmed by landing` 확인 |
| 착륙 후 20초 전체 스트림 | 통과, IMU/EV publisher 각각 1개 |
| 종료 후 두 번째 실행 | GUI·FAST-LIVO·PX4 ready 확인, 15초 전체 스트림 재수신 |

첫 20초 검사에서 IMU 2,147개, 점군 278개, RGB 277개, Livox 140개,
LIO 1,238개, EV 569개, PX4 local position 1,112개를 받았습니다.
이는 수신 개수이며 160Hz 실시간 처리 성능을 보장하는 값이 아닙니다.

제약과 확인한 실패:

- Ubuntu 단독 GPU는 이 WSL PC에서 실제 실행하지 못했습니다. 이전 Ubuntu 검증 기록과
  이번 native 실행기 테스트는 새 구조의 Ubuntu GPU 실행을 대신하지 않습니다.
- `verify.sh --ros` 전체 통과는 아닙니다. 실제 센서 메시지 직렬화와 Cyclone TCP
  전송은 통과했지만, WSL의 Fast DDS 별도 프로세스 검사에서 센서 수신이 0개였습니다.
  격리 bridge 검사에서도 DDS 연결/종료 대기가 발생해 중단했고, DDS gateway
  별도 프로세스 검사도 Cyclone endpoint 28초 대기 제한으로 실패했습니다.
- 두 번째 실행에서 기본 진단 참가자의 DDS 연결이 멈추었습니다. 기존 문서에도 기록된
  재연결 문제와 유사합니다. 사용하지 않는 Flight Manager peer `17450`을 진단용 XML에서
  제외한 뒤 15초 검사는 통과했습니다. 제품의 DDS 구성은 변경하지 않았으며, 이 우회가
  재연결 문제를 해결했다는 의미는 아닙니다.
- 실제 FAST-LIVO mapper는 두 번 모두 종료 시 PCD 저장 로그 뒤 exit code `-11`을
  남겼습니다. supervisor가 자식·포트를 회수하고 재실행은 가능했지만, mapper의 정상
  종료까지 검증한 것은 아닙니다. FAST-LIVO C++ 소스는 이번 구조 변경에서 수정하지
  않았으며 이 종료 오류의 원인은 확인하지 않았습니다.
- Windows 전역 키 입력 대신 Flight Manager에만 `ZAV_SIM_PLATFORM=native`를 지정해
  터미널 입력으로 비행했습니다. Isaac/PX4/센서/DDS는 Windows+WSL 구성이었습니다.

[이번 검증 원시 결과](validation/source-layout-20261026/)에 수신 결과·비행 위치·
소스 경로·설정 주기·검사 출력·실패 출력을 보관합니다.


## FAST-LIVO 종료·재실행 검사 (2026-10-26)

`run_fastlivo.sh`는 foreground Python 관리 프로세스를 사용합니다. 이 프로세스는
Ctrl+C/SIGTERM/SIGHUP을 처리하고, Linux subreaper로 ROS launch의 고아 자식까지
회수합니다. 정상 SIGINT 종료를 먼저 요청하고 남은 프로세스에는 SIGTERM/SIGKILL을
단계적으로 적용합니다. 정리하는 동안 실행 잠금을 유지하며, 잠금 inode는 삭제하지 않습니다.
실행기 잠금이 없어도 이 작업 공간의 mapper/bridge가 남아 있으면 중복 실행을 거부합니다.

실제 ROS 실행기에서 시작→Ctrl+C→재시작→Ctrl+C를 검사하여 TCP 17442/17446/17451
리스너와 자식 프로세스가 남지 않는 것을 확인했습니다. `check_fastlivo_shutdown.py`는
실제 PTY의 Ctrl+C, HUP/TERM, launch 조기 종료, 신호를 무시하는 별도 세션의 손자,
중복 실행 거부 및 동일한 세 TCP 포트의 반복 재사용을 검사합니다.
이 검사는 센서 수신·비행 성능의 재검증을 의미하지 않습니다.

RViz가 먼저 소유한 relay는 FAST-LIVO 종료 시 제거하지 않습니다. 이 경우 17442가
계속 열려 있는 것은 정상이며, FAST-LIVO가 소유한 17446/17451은 해제됩니다.
Windows Isaac 센서 서버 17441/17443/17444/17445도 계속 실행됩니다.

## Windows + WSL 실제 실행 검증 (2026-10-23)

Windows Isaac Sim 5.1.0 / RTX 4070 Ti와 WSL2 Ubuntu 22.04에서 실제 실행했습니다.
이 절은 Windows/WSL에서 수행한 별도 실행 검증 기록입니다.

실행을 막던 문제와 수정:

- FAST-LIVO가 먼저 센서 relay를 시작하면 임의 DDS 포트를 사용해 mapper가 센서를 발견하지 못했습니다. FAST-LIVO 실행기도 센서 publisher 고정 포트 17442를 사용합니다.
- Windows MAVLink 리스너가 열리기 전에 PX4가 접속하면 WSL의 TCP 연결이 SYN-SENT에 머물렀습니다. Windows `netstat`로 리스너를 확인한 뒤 PX4를 시작합니다. 시험 연결로 MAVLink 소켓을 소비하지 않습니다.
- LIO bridge는 진단 노드에서 발견할 수 있도록 TCP 17451을 사용합니다.
- FAST-LIVO 종료 대기에 상한을 두고 자식 프로세스에 실행기 잠금 fd를 넘기지 않습니다.
- 반복 시험 중 착륙 후 Isaac/PX4가 함께 멈추는 현상이 발생했습니다. Pegasus의 actuator 무기한 수신을 최대 0.2초로 제한했습니다. 정상 actuator가 도착하면 즉시 처리하므로 정상 lockstep 흐름은 유지합니다. 적용 후 연속 두 차례 이륙·착륙 및 착륙 후 전체 스트림 검사를 다시 통과했습니다. `check_px4_lockstep.py`는 실제 메서드를 추출해 메시지 중단·비 actuator 트래픽·정상 수신을 검사합니다.

검증 결과:

| 항목 | 실제 결과 |
| --- | --- |
| T 이륙 | 고도 1.95m에서 이륙 완료, 약 2m 호버 |
| 북쪽 1m·동쪽 1m 목표 | PX4 NED 위치 약 `(1.013, 1.031, -1.948)m` |
| 원점 복귀 | 약 `(0.014, -0.005, -1.980)m` |
| 터미널 ↑ 및 Space | 실제 전진 후 브레이크·호버, 시작 대비 약 4.23m 동쪽 이동 |
| L 착륙 | `Landing detected`, `Disarmed by landing` 확인 |
| 센서·FAST-LIVO·EV | 전체 수신 검사 통과, IMU/EV publisher 각각 1개 |
| RViz | RGB 영상 및 FAST-LIVO 궤적 표시 확인 |
| ROS CLI | 데몬 재시작 후 38개 조회했으나, 반복 진단 후 재조회 시 2개만 표시되는 현상 재발. 지속 해결은 미완료 |

터미널 방향키 시험은 flight_manager 프로세스에만 `ZAV_SIM_PLATFORM=native`를
임시 지정해 Windows 전역 키 읽기를 비활성화했습니다. Isaac/PX4/센서/DDS는
계속 Windows+WSL 구성이었습니다. Windows 전역 다중 키 입력과 Ubuntu 단독
GPU 실행을 이 시험으로 검증한 것은 아닙니다.

RViz를 켠 상태에서 12초간 측정한 wall-clock 수신률은 IMU 132.5Hz,
RGB 14.2Hz, 점군 14.3Hz, Livox 7.1Hz, LIO 83.1Hz, EV 34Hz였습니다.
설정 주기는 physics/IMU 200Hz, render/RGB 20Hz, Livox 10Hz이며,
실시간 렌더링 20Hz 확보까지 완료한 상태는 아닙니다.

기존 relay/mapper/EV의 처리가 계속되는 상태에서도 새 진단 참가자가 토픽을 발견하지 못하는 현상이 마지막 반복 검사에서 재현됐습니다. 저장된 `runtime-final.json`은 이 현상 발생 전 완료한 15초 수신 검사입니다. 새 DDS 참가자의 장기 재연결 안정성을 보장하는 기록은 아닙니다.

원시 검사 결과와 이동 위치는 [runtime-wsl-20261023](validation/runtime-wsl-20261023/)에 보관합니다.
이동 기록의 `manual-landed`는 수정 전 정지 시 수신이 없었던 실패 기록입니다.
`lockstep-*` 단계와 `runtime-final.json`은 대기 제한 수정 후의 재검증 결과입니다.

## Bootstrap 수정 검증 (2026-10-23)

Windows Isaac Sim + WSL2 환경에서 다음 명령을 직접 실행해 종료 코드 0과
`Summary: 7 packages finished`, `Install complete`를 확인했습니다.

```bash
./scripts/bootstrap.sh --platform wsl --isaac-windows-python /mnt/c/isaac-sim/python.bat
```

발생한 오류는 WSL 자체의 제한이 아니라 신규 설치 준비 및 의존성 선택 문제였습니다.
Ubuntu 단독 설치에서도 같은 패키지 구성이면 발생할 수 있습니다.

| 증상 | 원인 및 수정 |
| --- | --- |
| Livox `package.xml` 누락, 뒤따르는 `rosidl_interface_packages` 오류 | `scripts/prepare_ros.sh`에서 ROS 2 manifest를 준비합니다. CI의 새 clone에는 ROS 2 manifest 자체도 포함되어 있습니다. |
| Fast CDR `serializeArray`, `getState` 등의 컴파일 오류 | Agent의 private Fast CDR 2.x가 Humble 메시지 빌드에 섞였습니다. ROS 빌드는 Humble Fast CDR 경로를 명시합니다. |
| Sophus·Vikit `Eigen/Core`, `Eigen/Eigen` 누락 | Eigen3 CMake 타깃과 의존성 export를 명시했습니다. |
| 삭제된 DDS overlay 경로 경고 | ROS 빌드 전에 이전 prefix 환경변수를 초기화합니다. |
| SciPy `numpy.dtype size changed` | 사용자 NumPy 2.x와 apt SciPy의 ABI 충돌입니다. 프로젝트 환경은 `PYTHONNOUSERSITE=1`로 사용자 site-packages를 제외합니다. |

시스템 패키지와 rosdep 초기화·캐시가 이미 준비된 재실행은 apt 작업을 생략합니다.
빠진 패키지가 있거나 최초 설치이면 기존 sudo 설치 단계를 수행합니다.
개인 Python 패키지를 삭제하거나 전역 NumPy 버전을 변경하지 않습니다.

수정 후 `scripts/verify.sh`, 설치 점검, FAST-LIVO 실행 파일 조회,
Livox·PX4 메시지 직렬화 및 SciPy·cv_bridge 로딩을 확인했습니다.
이 설치 검증은 실제 Windows GPU 렌더링·비행 검증을 포함하지 않습니다.

## 배포 검증 기록 (2026-10-23)

배포 사본에서 원본 작업 디렉토리의 build/install을 사용하지 않고 확인했습니다. 외부 소스는 잠긴 커밋의 별도 checkout이며 Isaac Sim 5.1.0 설치만 기존 머신의 것을 사용했습니다.

| 항목 | 결과 |
| --- | --- |
| source syntax / 필수 7개 package XML / 개인 홈 경로 검사 / 설치 dry-run | 통과 |
| GitHub의 새 Ubuntu 22.04: 설치·고정 소스·rosdep·ROS 전체 빌드 (WSL 추가 수정 포함) | [전체 통과](https://github.com/ZenithFuturesLab/zav-simulator/actions/runs/39938818415) |
| 새 Pegasus 대상 배포, 재배포, 파일 변경 감지 | 통과 |
| 캘리브레이션·장착 IMU 회전·중력·lever arm 수학 검사 | 통과 |
| native/WSL DDS 선택, 공백 포함 Isaac 경로, Python ABI 분리, 종료 코드 전달 | 통과 |
| ROS 7개 패키지 새 빌드 (Livox SDK 없이 메시지 빌드) | 통과, upstream 컴파일 경고 있음 |
| PX4 v1.14.3 새 SITL 빌드 | 통과, 최소 Python 환경에서도 900개 단계 완료 |
| Agent·Fast DDS·Fast CDR·Micro XRCE client·Micro CDR 로컬 빌드/설치 | 통과, 의존 소스 SHA 검사 및 로컬 라이브러리 로딩 확인 |
| 실제 ROS 타입으로 IMU·RGB·pointcloud·Livox TCP 수신 | Fast DDS / Cyclone TCP 모두 통과 |
| flight_manager의 PX4 위치 피드백 QoS | 두 DDS 구현 모두 통과 |
| 자동 실행 LIO 브리지의 PX4 EV 변환 | 시험 메시지 80개, frame·시간·유한값 검사 통과 |
| 브리지 중복 실행 차단 및 부모 실행기 종료 시 동반 종료 | 통과 |
| Fast DDS ↔ Cyclone TCP 게이트웨이의 SensorCombined·TimesyncStatus·EV 경로 | 실제 프로세스 간 전달 통과 |

Ubuntu 실제 GPU 실행 기록은 아래에, Windows/WSL 실행 기록은 문서 상단에 정리했습니다. GitHub CI 자체는 GPU·비행을 실행하지 않습니다.

## 검사 재실행

설치 및 빌드 후 저장소 루트에서:

```bash
./scripts/verify.sh
# 실제 ROS 타입/전송/자동 launch 테스트 포함. 실행 중인 시뮬레이터를 먼저 종료하세요.
./scripts/verify.sh --ros
python3 scripts/doctor.py
```

ROS 검사는 격리 domain 49와 별도 테스트 포트를 사용합니다. 브리지와 relay는 사용자별 singleton이므로 실제 시뮬레이터와 동시에 시험하지 않습니다. `verify.sh --ros`는 비행·arm 명령을 보내지 않습니다.

## 실제 실행 확인

README의 네 명령을 실행하고 별도 터미널에서 환경을 source한 뒤:

```bash
source simulator/ros_setup.sh
python3 scripts/observe_runtime.py --duration 30
```

관찰기는 센서·FAST-LIVO·EV·EKF 위치 유효성과 중복 publisher를 검사하고 `.cache/runtime-check.json`에 기록합니다. 비행 명령을 보내지 않습니다. 통과 후 Flight Manager 터미널의 설명을 따라 이륙·이동·정지·착륙을 확인하세요. 네트워크·GPU 부하에 따른 실제 비행 성능은 각 컴퓨터에서 확인해야 합니다.

PX4 상단 콘솔에서는 `param show MPC_XY_VEL_MAX`가 새 설치에서 12이고 `param show EKF2_EV_CTRL`이 15인지 확인합니다. 기존 BSON이 있으면 저장값이 우선할 수 있습니다.

## 문제 해결

- `No module named rclpy` / 메시지 import 실패: ROS 노드는 Ubuntu의 Python 3.10에서 실행하고 `simulator/ros_setup.sh`를 source했는지 확인합니다. Isaac Python에서 ROS 노드를 실행하지 않습니다.
- `isaacsim` 없음 / 버전 불일치: bootstrap에 지정한 Isaac Python 경로를 확인합니다. 일반 시스템 Python이나 이전 컴퓨터의 venv를 지정하지 않습니다.
- PX4 속도 값이 다름: [파라미터 적용 순서](parameters.md)를 확인하세요. 텍스트 `.params`를 열었다고 자동 적용되는 것은 아닙니다.
- RViz의 GLib/Snap 오류: 프로젝트의 `run_mid360_rviz.sh`를 사용합니다. 실행기가 Snap GTK/GIO 라이브러리 환경을 정리합니다.
- `/fmu/out/*` 없음: `/tmp/micro_xrce_agent.log`, ROS domain ID, UDP 8888 Agent를 확인합니다. WSL에서는 `/tmp/zav-px4-gateway.log`도 확인합니다.
- 센서 없음: `/tmp/mid360-tcp-wsl.log`, `/tmp/zav_isaac_sim.log`, Windows/WSL 네트워크 모드와 포트를 확인합니다.
- EV 없음: `/LIVO2/imu_propagate`, `/drone0/mid360/imu`, PX4 `sensor_combined`와 `timesync_status`가 모두 필요한 브리지 입력입니다. duplicate bridge 에러이면 따로 실행한 브리지를 종료하세요.
- 재빌드 메모리 부족: `ZAV_BUILD_JOBS=1 ./scripts/build.sh`를 사용합니다. FAST-LIVO C++ 컴파일은 메모리를 많이 사용합니다.
- 조직 repo 생성 거절: GitHub CLI 인증 계정의 조직 접근/저장소 생성 권한, 조직 SSO 정책을 확인합니다. 저장소를 공개로 바꾸지 않습니다.

## Ubuntu 실제 실행 기록

배포본의 독립 PX4·ROS 빌드와 Pegasus 소스를 사용해 RTX 3080에서 Isaac Sim 5.1.0 GUI, FAST-LIVO2 자동 브리지, Flight Manager, RViz를 실행했습니다. PX4에 기존 BSON을 복사하지 않은 첫 부팅에서 `MPC_XY_VEL_MAX=12`, `MPC_VEL_MANUAL=12`, `EKF2_EV_CTRL=15`를 콘솔로 확인했습니다.

20초 관찰 결과: IMU 2,882개, Livox 164개, pointcloud 328개, RGB 320개, LIO odometry 1,806개, PX4 EV 765개, PX4 local position 1,643개를 수신했습니다. EKF의 `xy_valid`와 `z_valid`가 모두 true이고 EV·IMU publisher는 각각 1개였습니다. Flight Manager에서도 위치 수신이 확인됐으며 PX4가 `Ready for takeoff!`를 출력했습니다. [관찰 결과](validation/ubuntu-runtime.json).

이 기록은 disarmed 상태에서 센서부터 EKF까지의 연결을 확인한 것입니다. 이번 배포 검증에서 이륙·이동·정지 성능을 다시 측정하지는 않았습니다. 기존 비행 제어 소스와 설정을 유지했습니다. Windows/WSL의 별도 비행 검증은 문서 상단을 참고하세요.

통합 `run_zav_sim.sh`의 tmux 대시보드에서도 같은 경로를 재확인했습니다. Agent·Fast DDS·Fast CDR·Micro XRCE client·Micro CDR를 모두 저장소의 `.local/lib`에서 로딩했고, 20초간 EV 811개와 유효한 EKF 위치를 수신했습니다. [통합 실행 관찰 결과](validation/ubuntu-dashboard-runtime.json).

PX4 상단 pane에서 Ctrl+C를 입력했을 때 대시보드·Isaac·PX4·Agent가 함께 종료되는 것도 확인했습니다. PX4 pane 전용 실행기에 종료 trap을 추가해 기존에 PX4만 종료되던 동작을 수정했습니다.



## 설치 재현성 추가 확인

시스템 Python 패키지를 사용하지 않는 별도 가상환경에 `scripts/requirements-px4.txt`와 NumPy를 설치하고, 기존 `PYTHONPATH`를 제거한 상태에서 PX4를 별도 빌드 디렉토리에 다시 구성·컴파일했습니다. 900개 빌드 단계가 모두 통과했으며 Empy·Jinja2·Kconfiglib·genmsg·PyYAML·NumPy가 해당 가상환경에서 로드됨을 확인했습니다. 실제 설치 스크립트는 NumPy를 Ubuntu 패키지로 설치하고 PX4 전용 venv에서 시스템 패키지를 참조합니다.

공백을 포함한 임시 사용자 홈과 Windows Isaac 경로에서 WSL 설정 생성·재설치·셸 등록·Cyclone/domain 선택을 확인했습니다. USD 두 파일의 외부 asset 및 layer 참조가 없음을 확인했습니다. 이 검사는 실제 Windows GPU 실행을 대신하지 않습니다.

GitHub CI는 독립적인 Ubuntu 실행기에서 시스템 설치, 고정 소스 다운로드, 배포·경로 검사, rosdep 의존성 해결, 7개 ROS 패키지 빌드를 모두 통과했습니다. CI의 Ubuntu mirror 설정은 GitHub 실행기에만 적용하며 사용자 컴퓨터의 mirror는 바꾸지 않습니다.
