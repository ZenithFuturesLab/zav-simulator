# PX4 파라미터와 비행 설정

PX4 펌웨어 기본값 → SITL 시작 설정 → `10020_isaac_zav` airframe 기본값 → PX4에 저장된 사용자 파라미터 순으로 최종값이 결정됩니다. `param set-default`는 저장된 값을 덮어쓰지 않습니다. airframe의 일부 `param set` 시뮬레이션 설정은 매 시작 시 강제로 적용됩니다.

현재 컴퓨터의 BSON 파일은 기기별 calibration과 과거 값이 섞여 있어 배포하지 않습니다. 대신 기존 최종값을 airframe에 명시하여 **새로운 설치**에서 같은 프로파일이 적용되게 했습니다. `tuned.params`의 과거 3.0/1.5 속도 제한을 사용하지 않습니다.

| 파라미터 | 새 설치 기본값 |
| --- | ---: |
| EKF2_GPS_CTRL | 0 |
| EKF2_EV_CTRL | 15 |
| EKF2_EV_DELAY | 0 |
| EKF2_EV_NOISE_MD | 1 |
| EKF2_EVP_NOISE | 0.08 |
| EKF2_EVV_NOISE | 0.10 |
| EKF2_EVA_NOISE | 0.10 |
| EKF2_HGT_REF | 3 |
| EKF2_BARO_CTRL | 0 |
| EKF2_TAU_POS / EKF2_TAU_VEL | 0.10 / 0.10 |
| EKF2_MAG_TYPE | 5 |
| EKF2_MAG_DECL | -1.462524 |
| MPC_XY_P | 0.65 |
| MPC_XY_VEL_P_ACC / I_ACC / D_ACC | 4.0 / 0.15 / 0.30 |
| MPC_XY_VEL_MAX | 12.0 |
| MPC_XY_CRUISE | 1.5 |
| MPC_VEL_MANUAL | 12.0 |
| MPC_MAN_Y_MAX | 30.0 |

GPS는 계속 비활성화합니다. 정상 비행은 외부 odometry와 IMU로 높이를 추정하며,
기압계 센서 측정은 유지하되 융합은 끕니다(`EKF2_BARO_CTRL=0`).
점프 검출 시 MAVLink 파라미터 전환기가 값을 1로 바꾸고 실제 융합·수직 상태를 확인한 뒤 착륙을 요청합니다.
`EKF2_HGT_REF=3`은 유지하고, EV 중단 후 PX4 내부 높이 기준 fallback을 사용합니다.
새 SITL 부팅에서는 과거 failsafe가 저장한 1을 0으로 강제 초기화합니다.
PX4 v1.14에서 `COM_OBL_RC_ACT=4`는 Land, 5는 Hold입니다.

기타 MPC 설정과 기체 자세·각속도 제어, 모터 배치, hover thrust 등은 `simulator/px4/10020_isaac_zav`를 확인하세요. `simulator/current_ekf_mpc.params`는 그 파일의 EKF/MPC 기본값만 추출한 참고용 파일이며 자동 import하는 별도 입력은 아닙니다.

기존 설치에 저장된 값이 있으면 최종값이 다를 수 있습니다. 실행 중 PX4 상단 콘솔에서 확인하고 전체 유효값을 내보냅니다.

```text
param show MPC_XY_VEL_MAX
param show EKF2_EV_CTRL
param export /tmp/zav-current.params
```

`param export` 파일은 PX4 실행 프로세스의 파일시스템에 생성됩니다. 변경 후 재부팅에도 유지하려면 PX4 콘솔에서 `param save`를 사용합니다. `param reset_all`은 사용자가 저장한 다른 값까지 초기화하므로 설치 스크립트에서 실행하지 않습니다.

Flight Manager의 수동 이동 속도·가감속은 별도 ROS 파라미터입니다. `src/zav_control/zav_control/zav_flight_manager.py`의 현재 설정을 유지했으며 PX4 제한과 함께 동작합니다. LIO 브리지는 `run_fastlivo.sh`에서 자동 시작됩니다. 중복 실행 방지 lock을 사용하며 별도 Python 브리지를 동시에 실행하지 마세요.
