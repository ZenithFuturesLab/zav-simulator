# 운동 연속성으로 odometry 점프 감지와 착륙

현재 안전 판단은 **짧은 측정 시간 간격에서 위치 또는 자세가 갑자기 튀는지**만 봅니다.
이전 버전의 LiDAR 보정 신선도, EKF2 innovation/융합 상태, 수신 timeout,
시각 간격 초과에 의한 착륙 발동은 제거했습니다. CSV 저장·plot 기능은 유지됩니다.

## 판정 기준

`/LIVO2/imu_propagate`를 PX4 bridge에서 rate limit/좌표 변환 전에 검사합니다.
이전 정상 pose와 child-frame 선속도(현재 저장소 FAST-LIVO 구현)를 사용합니다.

- 위치 예측: `p_pred = p_prev + R(q_prev) × v_prev × dt`
- 위치 잔차: `||p_current - p_pred||`
- 위치 허용치: `0.5 + 0.3×dt + 0.5×20×dt²` m
- 자세 잔차: quaternion 상대 회전각. q와 -q는 같은 자세로 처리합니다.
- 자세 허용치: `20° + (4 rad/s)×dt`

`dt`는 odometry **메시지 측정 시각의 차이(초)**입니다. 실제 수신 시간 간격은
점프 판정에 사용하지 않습니다. 약 50 Hz(dt=0.02초)에서 기본 위치 허용치는 약
0.51 m, 자세 허용치는 약 24.6°입니다. 초기 SITL 값이며 실제 정상 기동 로그로 튜닝해야 합니다.

Armed 상태에서 어느 하나를 초과하면 **해당 샘플을 PX4 전달 전에 차단하고 즉시 착륙을 요청**합니다.
이상 샘플로 정상 기준을 옮기지 않으며, 한번 발동한 상태는 disarm이나 정상 데이터 수신으로
자동 해제되지 않습니다. 지상에서 FAST-LIVO 실행기와 flight manager를 모두 재시작하세요.
Disarmed 상태의 지도 초기화/좌표 reset은 착륙을 발동시키지 않습니다.

다음 조건은 점프로 판단하지 않습니다.

- 메시지 수신 중단, 시뮬레이션 정지 또는 느린 실행: 타이머만으로 착륙을 발동시키지 않습니다.
- 측정 간격이 기본 0.25초를 초과: 예측 비교를 건너뛰고 현재 유효 샘플로 기준을 새로 잡습니다.
- 중복·역행 시각, NaN/Inf, 잘못된 quaternion: 샘플만 버립니다.
- LiDAR 보정 정지 또는 EKF2 융합 상태 변화: 이 감시기의 착륙 조건에 포함하지 않습니다.

따라서 긴 단절 구간 사이의 점프나 부드러운 드리프트는 놓칠 수 있습니다.
기존 bridge의 좌표/측정 시각 유효성 검사와 PX4 자체 failsafe는 별도로 남아 있습니다.

## 착륙 경로

정상 비행에서는 **기압계 센서 측정은 계속하되 EKF2 융합만 끕니다**.
`EKF2_BARO_CTRL=0`, `EKF2_HGT_REF=3`, `EKF2_EV_CTRL=15`, `EKF2_GPS_CTRL=0`입니다.
flight manager의 이륙 준비 상태에는 파라미터 readback 0과 기압계 융합 비활성 확인이 필요합니다.

점프 검출 이후 순서:

1. 튄 샘플과 이후 odometry의 PX4 전달을 차단하고 latch를 유지합니다.
2. flight manager의 외부 trajectory/heartbeat와 재이륙을 차단합니다.
3. 파라미터 전환기가 MAVLink로 **EKF2_BARO_CTRL만 1로 변경**하고 PARAM_VALUE를 확인합니다.
4. 파라미터 응답만으로 전환 성공으로 보지 않습니다. 실제 기압계 융합 성공 샘플과
   PX4의 유효한 높이·수직 속도를 확인합니다.
5. 전환 확인 후 NAV_LAND를 요청합니다. ACK와 실제 모드를 구분하며,
   모드 전환이 되지 않으면 1초마다 다시 요청합니다.
6. 수평 위치를 잃으면 PX4가 AUTO_LAND에서 DESCEND로 전환하여 하강합니다.
   지면 감지 + disarm이 모두 확인되어야 착륙 완료입니다.

전환이 3초 안에 확인되지 않으면 오류를 기록하고 기압계 활성화를 계속 재시도하면서
최선의 NAV_LAND 요청을 합니다. 이를 전환 성공으로 표시하지 않습니다.
파라미터 연결 또는 센서가 실패하면 안전 착륙을 보장할 수 없습니다.
높이 기준 파라미터를 비행 중 강제로 바꾸지 않고, EV 차단 후 PX4 내부 높이 기준 fallback을 사용합니다.

기압계 상태와 높이 유효성은 **전환 확인용**이며 추가적인 odometry 발산 판정 기준이 아닙니다.
발산 판정은 운동 연속성 하나입니다. FAST-LIVO가 점프 없이 단순 정지하는 경우에는
이 감시기가 기압계 전환을 발동시키지 않습니다.

파라미터 전환기는 현재 SITL의 비어 있는 onboard-payload MAVLink 링크
(PX4 UDP 14280 → 로컬 UDP 14030)를 사용합니다. 표준 offboard/MAVSDK 포트 14540은 사용하지 않습니다.
이 링크를 다른 프로그램에서 사용 중이거나 실기체에 적용할 때는 별도 파라미터 통신 링크를
설정하고 `baro_param_connection`을 맞춰야 합니다. 다른 차량/다른 파라미터 응답은 인정하지 않으며,
비행 중 기압계 융합을 끄는 쓰기는 금지합니다. 한번 활성화 요청을 받으면 착륙 후에도 켜둡니다.
지상에서 FAST-LIVO 실행기를 재시작하면 전환기도 재시작되고, disarmed 상태에서 0으로 복귀시킵니다.

```bash
# 다른 MAVLink 파라미터 링크를 사용할 때만 변경
./run_fastlivo.sh baro_param_connection:=udpin:127.0.0.1:14030
```

전환기는 `.cache/px4-python/bin/python`의 pymavlink와 ROS를 사용합니다.
이 환경은 기존 `./scripts/build.sh`가 생성하며, launch가 전환기를 자동 실행합니다.
현재는 단일 EKF 인스턴스 0만 확인하며 multi-EKF 전환은 지원하지 않습니다.
실기체에서는 실제 기압계 또는 독립 높이 센서의 유효성이 필요하고,
DESCEND 중 횡방향 표류를 막을 수 없습니다. SITL의 짧은 호버·하강 결과가 모든 환경의
실기체 안전성을 보장하지는 않습니다.

## 실행과 상태 확인

`./run_fastlivo.sh`에서 기본 활성화됩니다. 실행 중 프로세스는 변경이 반영되지 않으므로
FAST-LIVO와 flight manager를 재시작하세요. 이전 EKF 진단 DDS export를 제거한 바이너리를
적용하려면 시뮬레이터/PX4도 재시작해야 합니다.

```bash
./run_fastlivo.sh

# 별도 터미널
source simulator/ros_setup.sh
ros2 topic echo /zav/lio_safety/status
```

상태에는 `comparison`, 측정 dt, 위치/자세 잔차와 허용치, `latched`, `reasons`,
착륙 ACK와 모드/완료 확인 결과가 표시됩니다.
기압계 전환은 `baro_phase`(`OFF_CONFIRMED` → `ACTIVATING` → `ACTIVE`),
`baro_parameter_value`, `baro_fusion_enabled`, `baro_activation_seconds`,
`baro_transition_confirmed`, `baro_transition_timed_out`으로 확인할 수 있습니다. `gap_rebaseline`은 시간 간격 때문에
비교하지 않았다는 뜻이며 착륙 발동이 아닙니다.

튜닝 예시:

```bash
./run_fastlivo.sh safety_position_slack:=0.7 safety_angle_slack_deg:=25
# 측정 간격 비교 범위 조정
./run_fastlivo.sh safety_prediction_horizon:=0.3
# 이 감시기 비활성화
./run_fastlivo.sh safety:=False
```

## 점프 주입 실험 (SITL 전용)

감시용 odometry 복사본에만 오류를 주입합니다. 원본 FAST-LIVO 결과를 직접 바꾸지 않지만
감시기가 실패하면 실제 착륙 명령을 보내므로 시뮬레이터 전용 그래프에서 실행하세요.
`--sitl`은 사용자 확인 표시이며 실제 FC 연결을 자동 검증하지는 않습니다.

```bash
# 기존 FAST-LIVO 실행을 종료한 뒤
./run_fastlivo.sh safety_experiment:=True

# flight manager로 이륙하고 READY 상태에서 호버한 뒤 별도 터미널
./run_lio_safety_experiment.sh trigger position-step --sitl \
  --log log/lio_position_step.jsonl
```

- `position-step`: X에 +3 m를 순간 추가합니다.
- `attitude-step`: world yaw에 +90°를 순간 추가합니다.

두 실험 모두 짧은 측정 간격에서 주입되어야 점프로 검출됩니다. 기본 오류 주입 지속 시간은
3초이며, 30초 동안 latch → 착륙 → disarm을 관찰합니다. 긴 측정 간격으로 비교가 생략되면
검출되지 않을 수 있으며 상태의 `comparison`을 확인하세요.
시험마다 착륙 후 실행기와 flight manager를 재시작합니다.
이전 `correction-drop`, `correction-freeze`, `ekf-reject`, `ekf-drop` 실험은 제거했습니다.

## 빌드와 테스트

```bash
source simulator/ros_setup.sh
# 착륙 ACK/지면 감지와 기압계 융합 확인 토픽 export
python3 scripts/configure_lio_safety_dds.py
cp simulator/px4/10020_isaac_zav src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10020_isaac_zav
make -C src/PX4-Autopilot -j2 px4_sitl_default
colcon build --symlink-install --packages-select zav_control

PYTHONPATH="$PWD/src/zav_control:${PYTHONPATH:-}" python3 -m pytest -q \
  src/zav_control/test/test_lio_safety.py src/zav_control/test/test_lio_safety_ros.py \
  src/zav_control/test/test_baro_parameter_link.py
```

자동화 검증은 정상 이동/회전, 작은 보정 허용, 위치·자세 점프, 느린 수신/긴 간격/입력 정지에서
발동하지 않는지, 잘못된 샘플 배제, latch 유지와 실제 bridge 발행 차단, 착륙 요청 재시도,
모드·착륙 완료 확인 및 점프 주입 relay를 포함합니다.
ROS 테스트는 기체 토픽을 고유 namespace로 remap하여 실제 PX4에 명령을 보내지 않습니다.
실제 비행 착륙 검증은 별도로 필요합니다.

## 자동 이륙부터 착륙 완료까지 실기동 검증

시뮬레이터와 FAST-LIVO를 실행한 상태에서, 기존 flight manager를 중복 실행하지 않고 사용합니다.
이 명령은 실제로 arm/이륙/착륙 명령을 보내므로 시뮬레이터 전용 그래프에서만 실행하세요.
`--sitl`은 사용자 확인 표시입니다. 실제 FC 연결을 자동으로 검증하는 장치는 아닙니다.

```bash
source simulator/ros_setup.sh
# 정상 이륙 → 약 2 m 호버 → 일반 자동 착륙
python3 simulator/tools/runtime/check_autoland.py --sitl --fault normal \
  --log log/autoland-normal.jsonl
# safety_experiment:=True로 실행한 FAST-LIVO에서 위치 점프 → failsafe 착륙
python3 simulator/tools/runtime/check_autoland.py --sitl --fault position-step \
  --log log/autoland-position.jsonl
```

`attitude-step`도 같은 방식으로 실행합니다. 각 점프 실험 후에는 착륙/disarm을 확인하고
FAST-LIVO 실행기를 재시작해 latch를 초기화해야 합니다. 이 검증기는 flight manager를 직접
생성하므로 별도 키보드 flight manager를 동시에 실행하지 않습니다.

JSONL에는 실제 simulator ground truth, PX4 추정 위치·고도, 비행 모드, 유효성 플래그,
착륙 감지, 안전 상태와 명령 ACK를 기록합니다. 공중에 있었던 것을 확인한 뒤,
실제 지면 높이 복귀 + landed + disarmed가 모두 확인되어야 PASS입니다.
정상 비행의 기압계 융합 OFF를 확인하고, 점프 시험은 전환 성공과 감시기의 `landing_complete` 확인도 기다립니다. 강제 disarm이나 kill로
성공 처리하지 않습니다. 로그 경로는 기존 파일을 덮어쓰지 않습니다.

기압계 상시 융합을 사용했던 이전 방식의 기록은 [이전 검증](validation/autoland-20261028/README.md)에 보존합니다.

현재 조건부 기압계 전환의 실제 비행 결과: [전환 검증 기록](validation/baro-switch-20261028/README.md).
