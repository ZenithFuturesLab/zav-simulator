# 검증과 문제 해결

## 배포 검증 기록 (2026-10-23)

배포 사본에서 원본 작업 디렉토리의 build/install을 사용하지 않고 확인했습니다. 외부 소스는 잠긴 커밋의 별도 checkout이며 Isaac Sim 5.1.0 설치만 기존 머신의 것을 사용했습니다.

| 항목 | 결과 |
| --- | --- |
| source syntax / package XML / 개인 홈 경로 검사 / 설치 dry-run | 통과 |
| 새 Pegasus 대상 배포, 재배포, 파일 변경 감지 | 통과 |
| 캘리브레이션·장착 IMU 회전·중력·lever arm 수학 검사 | 통과 |
| native/WSL DDS 선택, 공백 포함 Isaac 경로, Python ABI 분리, 종료 코드 전달 | 통과 |
| ROS 7개 패키지 새 빌드 (Livox SDK 없이 메시지 빌드) | 통과, upstream 컴파일 경고 있음 |
| PX4 v1.14.3 새 SITL 빌드 | 통과 |
| Agent·Fast DDS·Fast CDR·Micro XRCE client·Micro CDR 로컬 빌드/설치 | 통과, 의존 소스 SHA 검사 및 로컬 라이브러리 로딩 확인 |
| 실제 ROS 타입으로 IMU·RGB·pointcloud·Livox TCP 수신 | Fast DDS / Cyclone TCP 모두 통과 |
| flight_manager의 PX4 위치 피드백 QoS | 두 DDS 구현 모두 통과 |
| 자동 실행 LIO 브리지의 PX4 EV 변환 | 시험 메시지 80개, frame·시간·유한값 검사 통과 |
| 브리지 중복 실행 차단 및 부모 실행기 종료 시 동반 종료 | 통과 |
| Fast DDS ↔ Cyclone TCP 게이트웨이의 SensorCombined·TimesyncStatus·EV 경로 | 실제 프로세스 간 전달 통과 |

Agent 새 빌드 및 실제 GPU 실행의 최종 확인 결과는 아래 실행 기록을 참고하세요. Windows 장비에서의 GUI·GPU 종단 실행은 아직 수행하지 못했습니다. CI도 GPU·Windows 비행을 실행하지 않습니다.

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
source simulation/ros_setup.sh
python3 scripts/observe_runtime.py --duration 30
```

관찰기는 센서·FAST-LIVO·EV·EKF 위치 유효성과 중복 publisher를 검사하고 `.cache/runtime-check.json`에 기록합니다. 비행 명령을 보내지 않습니다. 통과 후 Flight Manager 터미널의 설명을 따라 이륙·이동·정지·착륙을 확인하세요. 네트워크·GPU 부하에 따른 실제 비행 성능은 각 컴퓨터에서 확인해야 합니다.

PX4 상단 콘솔에서는 `param show MPC_XY_VEL_MAX`가 새 설치에서 12이고 `param show EKF2_EV_CTRL`이 15인지 확인합니다. 기존 BSON이 있으면 저장값이 우선할 수 있습니다.

## 문제 해결

- `No module named rclpy` / 메시지 import 실패: ROS 노드는 Ubuntu의 Python 3.10에서 실행하고 `simulation/ros_setup.sh`를 source했는지 확인합니다. Isaac Python에서 ROS 노드를 실행하지 않습니다.
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

이 기록은 disarmed 상태에서 센서부터 EKF까지의 연결을 확인한 것입니다. 이번 배포 검증에서 이륙·이동·정지 성능을 다시 측정하지는 않았습니다. 기존 비행 제어 소스와 설정을 유지했으며 Windows GPU 실행도 별도 검증이 필요합니다.

통합 `run_zav_sim.sh`의 tmux 대시보드에서도 같은 경로를 재확인했습니다. Agent·Fast DDS·Fast CDR·Micro XRCE client·Micro CDR를 모두 저장소의 `.local/lib`에서 로딩했고, 20초간 EV 811개와 유효한 EKF 위치를 수신했습니다. [통합 실행 관찰 결과](validation/ubuntu-dashboard-runtime.json).

PX4 상단 pane에서 Ctrl+C를 입력했을 때 대시보드·Isaac·PX4·Agent가 함께 종료되는 것도 확인했습니다. PX4 pane 전용 실행기에 종료 trap을 추가해 기존에 PX4만 종료되던 동작을 수정했습니다.

GitHub 최초 CI에서는 Azure Ubuntu mirror의 다운로드가 약 50 KB/s로 지연되어 진단 후 중단했습니다. CI runner에 한해 공식 Ubuntu archive를 사용하도록 변경했습니다. 사용자 컴퓨터의 apt mirror 설정은 변경하지 않습니다. 설치기는 패키지 설정 대화상자를 기다리지 않도록 비대화식 설치와 네트워크 timeout/retry를 지정합니다.

새 CI의 rosdep 단계에서 Livox upstream `.gitignore`가 생성형 `package.xml`을 제외하는 누락을 발견했습니다. ROS 2 manifest를 명시적으로 버전 관리하고 필수 7개 패키지 manifest 존재 검사를 추가했습니다.
