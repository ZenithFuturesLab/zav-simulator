# 공개 OMMPC 구현과 PX4 기본 제어기 비교

기존 PX4 제어 시험 기록은 `docs/validation/zav-forest-20261105/`에 보존한다. OMMPC 시험은 `docs/validation/ommpc-forest-20261105/`에 별도 저장한다. 실제 ZAV/Isaac, MID360/RGB/IMU, FAST-LIVO2, SUPER와 원본 숲·목표를 사용하며, `perfect_drone_sim`은 사용하지 않는다.

SUPER 논문은 상위 OMMPC가 각속도·추력을 계산하고 PX4가 하위 제어를 수행한다고 설명하며, 로터 항력 보상을 포함한다. 여기서는 사용자가 지정한 [공개 구현](https://github.com/OliverWu515/ommpc_controller)의 `feature/drag_compensation` commit `29cf38acf6a9a6fb9627c189d6e66afbcc605244` 코어를 ROS2에 연결한다. 논문의 비공개 실험 코드와 동일하다고 주장하지 않는다.

## 준비

```bash
cd /home/goat/zenith_ws
./scripts/build_zav_ommpc.sh
```

OSQP 0.6.3을 작업 폴더의 `.cache/ommpc-deps`에 설치한다. 전역 ROS1/MAVROS 환경을 추가하지 않는다. PX4에는 호버 추력 측정용 읽기 전용 `/fmu/out/vehicle_rates_setpoint` 발행을 추가한다. 제어 알고리즘과 모터 모델은 바꾸지 않는다.

## 실행

각각 다른 터미널에서 실행한다. 이륙은 마지막 임무 실행기가 담당한다.

```bash
./scripts/run_zav_forest.sh high_speed
```
```bash
./run_fastlivo.sh
```
```bash
./scripts/run_zav_forest_planner.sh high_speed 2
```
```bash
./scripts/run_zav_ommpc.sh
```
```bash
./scripts/run_zav_forest_mission.sh high_speed --controller ommpc --max-tracking-speed 2.2
```

마지막 프로세스가 실제 flight_manager다. 별도 manager를 중복 실행하지 않는다. dense에서는 네 번째 터미널을 `./scripts/run_zav_ommpc.sh -p max_yaw_rate:=1.0`으로 실행한다. 나머지는 각각 `dense`, `dense 1`, 마지막 `dense --controller ommpc --goal-timeout 360 --timeout 900`을 사용한다. dense 전체 임무 통과 여부는 실제 시험 기록에서 확인한다. 실험 중 `Ctrl+C`는 정상 착륙을 요청하고 disarm을 기다린다.

PX4 기본 제어기로 돌아가려면 OMMPC 노드를 종료하고 마지막 명령의 `--controller ommpc`를 생략한다. 비교를 반복할 때는 시뮬레이터부터 재시작해 같은 원본 지상 출발 위치로 되돌린다.

## 제어 경로와 비교 조건

- 위치 추정과 장애물 지도는 FAST-LIVO2다. OMMPC는 추종 제어기를 바꾸며 추정기를 바꾸지 않는다. ground truth는 초기 목표 좌표 변환과 사후 검사에만 사용한다.
- 기존 PX4 경로: SUPER P/V/A → PX4 위치·속도 제어 → 자세·각속도 제어 → 모터.
- OMMPC 경로: SUPER의 확정된 전체 7차 다항식 → OMMPC P/V/자세 계산 → 각속도·추력 → PX4 각속도 제어 → 모터. 임시 전진 경로가 아니라 실제 committed/backup 궤적을 전달한다.
- `/zav_super/committed_trajectory`는 시작 시각, 조각별 duration, 축별 내림차순 8개 계수를 전달한다. 비어 있는 메시지는 이전 궤적 무효화다.
- OMMPC는 원본 코어의 20-step/0.01s horizon을 사용하고 100Hz로 계산한다. flight_manager가 기존 50Hz 주기에 출력 유효성을 검사하고 유일하게 PX4 입력을 발행한다.
- 위치·속도는 기존과 같은 LIO IMU 기준점으로 오차를 계산한다. 기체 자세에서는 +22° IMU 장착 회전을 제거한다. 원본 코어의 fixed-yaw=0을 유지하면서 계산 세계 좌표계를 원하는 yaw만큼 회전해 기존 manager의 yaw 정책을 따른다. dense의 각속도 상한은 1.0rad/s로 제한해 목표 yaw 변화율과 맞춘다. 출력 FLU 각속도는 FRD `[wx,-wy,-wz]`, 추력은 `[0,0,-u]`로 전달한다.
- 정상 PX4 이륙 후 실제 호버 출력 75개로 추력 스케일을 초기화한다. 저장소의 기본 hover_percentage를 이 기체에 그대로 적용하지 않는다. 이후 원본 코어의 IMU 기반 추력 적응을 사용한다.
- ZAV의 실제 선형 항력은 body 축 `[0.50,0.30,0]`kg/s다. 공개 코어는 수평 등방 모델을 지원하므로 평균 `0.4`kg/s, 수직 `0`, mass `1.99`kg을 사용한다. 이 모델 차이는 남는다.
- IMU가 20Hz `/clock`보다 조금 앞서는 경우 측정 시각과 같은 시각의 다항식 reference를 사용한다. LIO stamp를 바꾸지 않으며 오래된 측정만 짧게 전방 예측한다. 명령 스트림과 다항식이 5cm 이내로 일치하는지도 확인해 서로 다른 commit을 섞지 않는다.
- 이륙·착륙은 기존 PX4 경로이며, 추종 중에만 body-rate Offboard로 전환한다. 입력 freshness, 0.5m 위치 오차, 명령 속도 제한, LIO safety와 정상 착륙 보호는 유지한다. 오래된/범위를 벗어난 MPC 출력은 위치 유지로 전환한다.
- 같은 환경에서도 재계획 결과와 센서 잡음에 따라 경로가 달라진다. RMS/P95 오차와 성공 여부는 반복 횟수 및 시간 예산과 함께 비교해야 하며 한 번의 성공으로 우월성을 일반화하지 않는다.

## 사후 비교

`simulator/benchmarks/zav/compare_controllers.py`에 원시 `.jsonl` 또는 `.jsonl.gz` 로그들을 전달한다. 모든 명령을 읽은 뒤 LIO 측정 시각에 맞춰 보간하므로, 실행 중 최신 IMU 상태가 명령보다 약간 앞서는 경우도 같은 기준으로 비교한다. 이동 구간은 reference 속도 0.2m/s 이상이다. 실제 body의 나무 복셀 거리, PhysX 접촉, 숲 범위, 원본 목표와 최종 착륙도 기존 독립 검사로 확인한다.


## 측정 결과

[OMMPC 비교 기록](validation/ommpc-forest-20261105/README.md)에 원시 로그와 지표를 보관한다. 고속 숲에서는 계획 2m/s로 두 번 왕복했고, 시각 정렬을 보강한 시험의 이동 RMS는 약 4.54cm였다. 보존한 PX4 기본 제어기 두 번의 RMS는 15.49cm/16.29cm였다. 경로가 동일하지 않으므로 반복 조건을 함께 해석해야 한다. dense 전체 왕복은 아직 미통과다. 실물 치수를 반영한 충돌체·팽창 변경은 [별도 footprint 실험](zav-footprint-audit.md)으로 구분한다.
