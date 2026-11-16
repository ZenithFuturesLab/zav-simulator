# SUPER → PX4 SITL 추종과 backup 복구

zav_sim의 ENU 정렬 및 Mid360 +22° 장착을 전제로 하는 실험용 연결이다.
실기체용 설정이 아니다. 2026-11-04: 궤적 추종, backup 실행 유지,
정지 후 동일 목표 재계획까지 구현했다. 2026-11-05 Warehouse SITL에서 정상 추종,
실제 backup 복구, 중단·자동 착륙 시험을 통과했다.
[실제 비행 검증 결과](validation/super-stability-20261105/README.md)를 참고한다.

## 실행

각 터미널에서 기존 env/zav_sim 환경과 workspace setup을 사용한다.

```bash
cd /home/goat/zenith_ws
source simulator/environment.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
```

1. zav_sim 실행 후 Play. `./run_fastlivo.sh` 실행.
2. 기존 flight_manager가 있다면 착륙 후 종료하고 다음으로 실행한다. PX4 setpoint 발행기는 하나만 사용한다.

```bash
ros2 run zav_control flight_manager --ros-args -p super_tracking:=true -p use_sim_time:=true
```

3. 기존 replanner를 종료하고 다음으로 실행한다.

```bash
ros2 launch zav_super zav_super_replanner.launch.py \
  rviz:=true replan_mode:=tracking_guarded replan_rate_hz:=1.0 \
  enable_tracking_commands:=true
```

4. `/clock`, `/aft_mapped_to_init`, `/fmu/out/vehicle_local_position` 갱신 및
   `/zav/lio_safety/ready=true` 확인. flight_manager에서 T로 이륙하고 호버링을 기다린다.
5. flight_manager에서 2를 누른 뒤 RViz에서 가까운 자유 공간에 새 목표를 지정한다.
   처음에는 수평 0.5~1m, 현재 LIO 고도와 같은 높이를 사용한다.
   RViz 2D 목표 z=0은 camera_init z=1.5로 바뀐다. 높이를 명시하려면
   `/zav/goal_pose` PoseStamped를 camera_init 좌표로 발행한다.
6. 일반적인 재계획 실패에는 2를 다시 누르거나 목표를 재발행할 필요가 없다.
   기존 committed 궤적을 실행하며 재시도하고, backup 종료 후 동일 목표로 다시 계획한다.
7. Space: 추종 중단·호버링. 1: 수동 복귀. L: 착륙.
   안전 차단으로 flight_manager가 latch되었다면 원인 해소 후 2와 새 목표가 필요하다.

## 실패 및 복구 동작

- `ReplanOnce=FAILED`: 기존 committed snapshot과 RViz 경로를 보존하고 재계획 주기마다 재시도한다.
  기존 snapshot에는 SUPER가 연결한 일반 궤적과 backup 궤적이 이미 들어 있다.
- backup에 진입하면 같은 다항식을 시간에 맞춰 계속 실행한다. 성공한 새 계획은 snapshot을 교체한다.
- 기존 궤적이 끝나면 종점 위치 + 속도 0 명령을 50Hz로 유지한다. 명령 timeout을 이용해 정지하지 않는다.
- LIO 속도 0.2m/s 이하가 0.5초 지속되어야 `PlanFromRest`를 호출한다.
- 목표와 거리가 0.2m 이하이면 `GOAL_REACHED_HOLDING`, 아니면 같은 목표에 대해 정지 상태 계획을 시도한다.
- 정지 상태 계획도 실패하면 종점 호버 명령을 유지하며 1초 간격으로 재시도한다.
  목표 무효가 확인되면 `INVALID_GOAL_HOLDING`으로 자동 재시도를 중단한다.
- 이동 중 새 목표는 기존 committed 궤적 종료·정지 확인 뒤 계획한다.
  이는 움직이는 상태에서 rest-to-rest 궤적으로 바뀌는 불연속을 피하기 위한 ZAV 동작이다.
- `EMER`, 잘못된 committed 궤적, odometry 유실, 추종 오차 초과, 시계 역행은
  snapshot을 제거하고 자동 재출발하지 않는다. flight_manager watchdog/LIO 안전 처리가 계속 적용된다.
- preview 기본값은 제어 샘플을 발행하지 않으며 기존 실패 시 중단 동작을 유지한다.

## 상태 확인

```bash
ros2 topic echo /zav_super/replan_status
```

대표 상태:

| 상태 | 의미 |
|---|---|
| REPLAN_FAILED_KEEP_COMMITTED / RETRYING_WITH_COMMITTED | 기존 궤적을 유지하며 재계획 재시도 |
| EXECUTING_BACKUP | 기존 committed의 backup 구간 실행 |
| WAITING_FOR_REST | 종점 도달 또는 초기 목표 수신 후 실제 정지 확인 |
| REST_PLAN_FAILED_RETRYING / REST_PLAN_RETRY_WAIT | 종점 호버 유지, 동일 목표 재시도 |
| PLAN_FROM_REST_SUCCESS | 정지 상태 계획 성공, 새 궤적 실행 |
| GOAL_REACHED_HOLDING | 목표 도달 후 종점 호버 유지 |
| *_NEW_GOAL_REQUIRED | 안전 차단, 자동 재출발 안 함 |

## 제어 연결과 제한

- `/zav_super/tracking_command` Odometry의 pose와 twist는 모두 camera_init 좌표다.
  별도 50Hz 콜백이 snapshot을 평가하므로 최적화가 직접 명령 타이머를 점유하지 않는다.
- super_tracking=true인 flight_manager는 `/zav/goal_pose`, `/zav/cmd_vel`을 직접 구독하지 않는다.
- 동시각에 가까운 LIO/PX4 피드백으로 위치 오차를 PX4 로컬 원점으로 옮기고
  ENU 오차/속도를 NED로 변환한다. yaw는 유지한다. yaw 궤적·가속도 feedforward는 미구현이다.
- 명령·LIO·PX4 수신 0.25초, ROS stamp 0.3초, LIO safety ready 수신 1초 검사와
  추종 오차 0.5m 제한은 유지한다.
- snapshot lease는 최적화 성공 시각이 아니라 정상적인 odometry/추종 오차 감시 tick으로 갱신한다.
  따라서 일반 실패가 반복되어도 명령은 이어지며, 감시 콜백이 2.5초 멈추면 발행을 중단한다.
- 정지 끝점이 있는 committed 궤적만 사용한다. 정지 종단 속도 검증에 실패하면 차단한다.
- 정지·재시도는 동일 목표를 사용한다. 막힌 목표가 반드시 도달 가능해지는 것은 아니다.
- 이전에 생성된 backup을 실행하는 것이 이후 나타난 동적 장애물에 대한 충돌 회피 보장은 아니다.

## 검증과 남은 작업

구현 테스트는 실제 노드 tick과 명령 발행을 사용하되 센서/플래너 결과를 주입한다.
격리 ROS domain에서 재계획 반복 실패 → backup 종료 → 정지 확인 → 정지 상태 계획 실패·재시도 →
재출발 → 목표 호버링 동안 50Hz 명령 651개의 연속성을 검사한다.
별도 테스트는 실제 CmdTraj의 일반/backup 연결, 종점 유지, 감시 lease, 시계 리셋 및 정지 gate를 검사한다.
기존 실제 backup 최적화기의 가속도 회귀 테스트도 유지한다.

실제 Warehouse SITL에서 정상 3회 비행·10개 목표, backup 실패 복구, 플래너/LIO 중단,
수동 정지, LIO 점프 자동 착륙을 검증했다. 다른 맵·속도·실기체는 별도 검증이 필요하다.

### Backup 가속도 설정

replanner의 `backup_traj.penna_ts=1000`은 제동 시작을 지나치게 늦추는 비용을 줄인 설정이다.
가속도 상한은 1.0m/s², 기존 허용 판정은 유지했다. 실제 optimizer 회귀 테스트에서
원래 설정이 실패하던 조건을 포함한 9개 제동 조건이 통과했다.

### 2026-11-05 제어 타이머 수정

flight_manager 제어 타이머는 STEADY_TIME 기준 50Hz로 동작한다. 노드의
`use_sim_time=true`는 궤적 stamp 검사에 계속 사용하되, 약 20Hz인 `/clock`이
PX4 heartbeat/setpoint 주기를 제한하지 않도록 분리했다. 적용하려면
flight_manager를 착륙 후 재시작해야 한다. 오차 차단 시 목표·측정 ENU 위치와
메시지 수신 나이가 로그에 추가된다. 추종 오차 및 timeout 제한은 그대로다.


### 2026-11-05 지연·속도·실행 허용 연결

- 플래너의 오차 보호는 LIO 측정 시각의 기준 궤적과 비교한다.
- flight_manager는 공분산 계약이 유효한 LIO 속도로 지연 위치를 명령 시각에 맞춘다.
  stamp를 바꾸지 않으며 0.3초 시각 범위와 0.5m 차단 한계는 유지한다.
- ROG-Map은 child-frame twist를 world 속도로 회전해 실제 정지 확인에 사용한다.
- `/zav_super/execution_enabled`가 fresh true일 때만 플래너가 추종을 실행한다.
  flight_manager를 먼저 실행하고 T → 호버 → 2 → 새 목표 순서로 사용한다.
  착륙·수동 중단·차단은 이전 목표도 취소한다. 이전 바이너리를 혼용하지 않도록
  수정 적용 후 flight_manager와 replanner를 모두 재시작한다.
- FAST-LIVO의 IMU 대기 버퍼는 샘플별 dt로 적분한다. 착륙 접촉의 짧은 충격이
  전체 callback 지연 시간만큼 과장되는 문제를 수정했다.
