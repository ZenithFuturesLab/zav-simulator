# ZAV / PX4로 SUPER 공식 숲 환경 비행

공식 SUPER의 숲을 **기존 Isaac ZAV + PX4 SITL + MID360 + FAST-LIVO + zav_super + flight_manager**로 비행한다. `perfect_drone_sim`은 실행하지 않는다.

현재 **high_speed 환경은 계획 2m/s로 원본 200m 왕복을 실제 PX4에서 통과**했다. **dense는 환경과 임무 실행을 구성했지만 전체 왕복 미통과인 실험 설정**이다. 원본 8/25m/s 성능을 재현했다는 뜻은 아니다. [실제 시험 기록](validation/zav-forest-20261105/README.md)을 함께 확인한다.

## 실행

현재 실행 중인 기체가 있으면 착륙·disarm 후 시뮬레이터와 FAST-LIVO, replanner, manager를 종료한다. 각 터미널의 ROS domain은 같아야 한다. 기존 `simulator/environment.sh` 설정을 사용한다.

터미널 1 — 숲을 선택해 기존 zav_sim 실행:

```bash
cd /home/goat/zenith_ws
./scripts/run_zav_forest.sh high_speed
```

더 조밀한 숲은 `high_speed` 대신 `dense`다. 이 실행기는 `ZAV_BENCHMARK`를 tmux/Isaac에 전달해 기본 Warehouse 대신 해당 숲을 로드한다. 기체, 모터 모델, 센서, PX4 연결은 기존 ZAV 설정이다. 기본 `./run_zav_sim.sh`는 기존 Warehouse다.

터미널 2 — 기존 FAST-LIVO 실행:

```bash
cd /home/goat/zenith_ws
./run_fastlivo.sh
```

터미널 3 — ZAV용 SUPER 플래너 및 RViz:

```bash
cd /home/goat/zenith_ws
./scripts/run_zav_forest_planner.sh high_speed 2
```

마지막 `2`는 실제 왕복 시험을 통과한 계획 최대 속도 2m/s다. 생략 시 실행기 기본값은 1m/s다. 숲 전용 프로파일은 재계획 3Hz, forward look-ahead 0.35초를 사용한다. 선택한 숲은 터미널 1과 같게 한다. 원본 high-speed 예제의 25m/s와는 다른 ZAV 검증 속도다.

터미널 4 — 자동 이륙·원본 목표 왕복·착륙:

```bash
cd /home/goat/zenith_ws
./scripts/run_zav_forest_mission.sh high_speed --max-tracking-speed 2.2
```

dense 실험은 터미널 1에서 `dense`, 터미널 3에서 `dense 1`, 터미널 4에서 `dense --goal-timeout 480 --timeout 1100`을 사용한다. dense 전체 왕복은 아직 미통과다.

**이 프로세스가 실제 flight_manager를 생성하므로 다른 flight_manager를 동시에 실행하지 않는다.** 준비 상태와 단일 PX4 publisher를 검사한 뒤, 실제 T/2/L 동작으로 이륙·추종·착륙한다. 처음 2m 이륙 후 원본 시작 고도 1.5m로 맞추고 다음 body/world 목표를 순서대로 보낸다.

```
(0, -50, 1.5) → (0, 50, 1.5) → (0, -50, 1.5)
```

dense에서는 이동 방향으로 기수를 천천히 돌리는 옵션을 기본 적용한다(최대 1rad/s). 기존 기본 flight_manager의 고정 yaw는 유지하며 이 옵션은 숲 실험에서만 켠다.

원본 전체 100m 편도·200m 왕복이다. 목표는 FAST-LIVO의 `camera_init` 좌표로 변환해 `/zav/goal_pose`에 발행한다. 원본 목표 도달 반경 1m를 Isaac의 실제 body 위치로 독립 확인하고, ZAV 내부에서는 정지 후 LIO 목표 오차 0.2m 미만도 요구한다. 완료하면 정상 착륙하고 disarm을 기다린다. `PASS`는 전체 임무와 착륙의 성공이며, 중간에 원본 숲 범위를 벗어나면 별도 사후 검증을 통과하지 않는다.

다시 실험하려면 시뮬레이터부터 재시작해 원본 출발 위치로 복귀시킨다. 비행 중 `Ctrl+C`를 누르면 검증 드라이버가 정상 착륙을 요청하고 disarm을 기다린다. 시뮬레이터는 착륙 전에 닫지 않는다. 기본 goal timeout은 240초다. 더 느린 dense 시험에서는 `--goal-timeout 360 --timeout 900`을 임무 명령 뒤에 붙일 수 있다. 로그는 `/tmp/zav-forest-<profile>-<time>.jsonl`에 남는다.

## 지도 변환 및 좌표

- 원본 commit: `2ad3419c127a617c6d7df6925e81a14175a9c096`.
- dense: `random_map_2_26609.pcd`; high_speed: `random_map_24_6635.pcd`.
- 유한 점 중 `z>0.2m`의 나무 점을 dense 5cm / high_speed 10cm 복셀의 외곽 사각 면으로 바꿔 **RTX LiDAR/RGB 렌더링과 PhysX 충돌에 같은 메시**를 사용한다. 나무 위치나 목표를 임의로 다시 생성하지 않는다.
- 원본 PCD는 점군이며 solid/충돌 메시를 제공하지 않으므로 표면의 완전한 동일성은 보장하지 않는다. 한 점에서 그 복셀 내 임의 위치까지 dense 최대 약 0.087m / high_speed 약 0.173m이며, 실제 지형의 저면 `z<=0.2`는 평탄한 `z=0` 착륙 바닥으로 대체했다.
- 시작 body 위치 `(0,-50,0.1)`, yaw `1.5741rad`. 지상 이륙이 필요한 PX4라 원본의 공중 초기화와 다르다.
- 실제 MID360 장착 위치와 +22° pitch는 그대로다. 원본의 178° 수직 FOV를 흉내 내기 위해 ZAV 센서를 바꾸지 않았다.
- 지상 정지 상태의 실제 IMU 자세와 LIO 자세로 `world_from_camera_init`을 한 번 구한다. 실제 body 목표에는 IMU lever arm을 반영한다. **ground truth는 좌표 변환 및 검증에만 사용하며, 비행 중 플래너 지도/상태 및 PX4 외부 비전에는 FAST-LIVO 추정만 사용한다.**

준비 결과는 `.cache/zav_benchmarks/<profile>/forest.npz`와 `scene.json`이다. 원본 준비 스크립트와 변환기가 최초 실행 때 자동 실행된다. 직접 재생성하려면:

```bash
python3 scripts/prepare_super_benchmarks.py
/usr/bin/python3 simulator/benchmarks/zav/prepare_forest.py dense
/usr/bin/python3 simulator/benchmarks/zav/prepare_forest.py high_speed
```

## ZAV 전용 플래너 차이

원본의 mass 1.64kg·반경 0.2m·8/25m/s 제약을 ZAV에 그대로 적용하지 않는다. ZAV mass 1.99kg·두 프로파일 모두 IMU 기준 반경 0.43m와 0.6m inflation을 사용해 frontend와 회랑의 여유 조건을 맞춘다. 원본 15m × 110m 숲 안에서, 측면 1m씩의 경계 여유를 둔 110m × 13m 고정 계획 지도를 사용한다. 환경의 나무와 원본 목표 위치는 그대로다. `load_pcd_en=false`이며 지도는 실제 센서에서 만들어진다. dense의 exp_traj 비용과 receding distance는 원본 dense YAML에서 가져오고, backup 비용과 동적 경계는 ZAV의 저속 제약에 맞춘 값을 사용한다. dense look-ahead는 15m이며 A* 탐색 격자도 이에 맞춰 확대한다. ROG가 가상 ground/ceiling을 inflation만큼 안쪽으로 이동하는 것도 고려해 dense의 실제 body 중심 높이 범위를 약 0.43..3.0m로 맞춘다.

추종 상태와 ROG-Map의 로봇 상태에는 FAST-LIVO `/LIVO2/imu_propagate`를 사용하고, 숲은 원본처럼 정적 occupied-only 맵 갱신을 사용한다(`raycasting.enable=false`, 수신된 모든 실제 장애물 점을 지도 범위 안에서 누적). 기존 Warehouse의 raycasting 프로파일은 변경하지 않는다. 이 모드는 움직이는 장애물 제거를 검증한 설정이 아니다.

`/cloud_registered_full`은 카메라 FOV로 잘라내기 전의 실제 FAST-LIVO 전체 LiDAR 점군이다. 카메라를 켰을 때 `/cloud_registered`는 RGB 표시용으로 카메라 시야 밖의 점을 제거하므로 장애물 지도 입력에 사용하지 않는다. flight_manager는 수신 버퍼에서 명령 시각 이전의 상태를 골라 epoch를 맞춘다. 기존 mapped odometry는 기본 ZAV 실행의 기본값으로 남는다. 기존 시간 보호와 0.5m 추종 오차 제한을 늘리지 않는다.

## 사후 독립 검사

```bash
/usr/bin/python3 simulator/benchmarks/zav/analyze_flight.py \
  /tmp/zav-forest-high_speed-YYYYMMDD-HHMMSS.jsonl --output /tmp/flight-report.json
```

실제 body 중심 경로를 2cm 이하 간격으로 보간하고, 원본에서 만든 모든 나무 복셀과의 거리를 검사한다. 최근접 복셀 중심 거리에서 복셀 외접구 반경과 표본 간 반간격을 빼는 보수적인 표면 거리 하한을 쓴다. 메타데이터의 body 중심 외접구(현재 0.32m) 여유, 숲의 원래 XY 범위, 정확한 세 목표 도달, PhysX 나무 접촉 기록, 최종 정상 착륙을 함께 확인한다.

기본은 1m/s다. 2m/s 단계 시험은 플래너를 `./scripts/run_zav_forest_planner.sh high_speed 2`로 실행하고, 임무 드라이버에 `--max-tracking-speed 2.2`를 붙인다. 기존 1.5m/s 보호 한계의 opt-in 변경이며, 0.5m 추종 오차 제한은 그대로다. 원본 8/25m/s는 이 ZAV 프로파일의 검증 속도가 아니다.

실제 시험 결과와 한계는 [검증 기록](validation/zav-forest-20261105/README.md)에 정리한다.

## 원본 LiDAR 시야 비교 옵션

기본은 실제 ZAV의 MID360 근사 모델이다. 원본 벤치마크의 넓은 관측 범위를 비교할 때만 다음 옵션을 쓴다.

```bash
./scripts/run_zav_forest.sh dense official
```

이 모드는 RTX LiDAR의 수직 시야를 178°, 128채널·수평 약 0.72°·회전 10Hz로 만든다(range: dense 15m / high_speed 70m). 원본의 각 해상도를 그대로 적용하면 현재 Python 센서 전송 경로에 backlog가 생겨, 이 비교 옵션은 포인트 수를 제한한다. 수직 시야만 원본에 맞추며 각 해상도는 동일하지 않다. MID360 모델이 아니므로 해당 결과는 기본 MID360 결과와 구분해야 한다. PX4, ZAV 동역학, 센서 장착/IMU 및 FAST-LIVO 추정은 그대로 사용한다. `perfect_drone_sim`이나 가짜 odometry는 실행하지 않는다. 메타데이터의 `lidar_mode`에 `official`로 기록한다.

## 기체 충돌체와 반경

기존 USD의 활성 충돌체는 반경 0.08m·높이 0.02m의 착륙용 원통 하나다. 숲 시험에서는 시각 모델 전체를 감싸는 반경 0.27m·높이 0.25m 원통 프록시를 추가한다. 실제 모델의 593개 메시 좌표를 매번 검사해 프록시 내부에 들어오는지도 확인한다. 원본 USD 파일과 명시된 1.99kg 질량·관성은 변경하지 않는다.

실제 시각 모델의 body 기준 최대 반경은 약 0.249m, 새 프록시의 body 기준 외접구는 약 0.309m다. 플래너 기준점은 body가 아닌 장착 IMU이므로, IMU 오프셋을 포함하면 프록시 반경은 약 0.404m다. 따라서 **회랑 반경은 기존 0.43m를 유지한다.** 0.32m로 줄인 dense-06 시험은 접촉으로 실패해 채택하지 않았다. frontend의 0.6m 팽창은 추종·지도 오차 여유로 유지한다. 사후 검사는 이 0.32m 구와 실제 나무 충돌 이벤트를 함께 확인한다. 이전 high-07은 프록시 추가 전 결과지만 독립 거리 하한 0.572m가 전체 기체 외곽보다 충분히 컸다.

## 가속도 feed-forward

숲 임무 실행기는 `super_acceleration_feedforward:=true`를 사용한다. SUPER의 committed/backup 위치 다항식에서 위치·속도·가속도를 같은 시각에 샘플링한다. 가속도는 `/zav_super/tracking_acceleration`의 `AccelStamped`로 전달하고, manager는 위치 명령과 stamp가 일치하는 값만 ENU→NED 변환해 PX4 `TrajectorySetpoint.acceleration`에 넣는다. 중력은 포함하지 않는다. 일치하는 가속도 공백 0.25초, 최대 norm 3m/s², 기존 위치 오차 0.5m 보호를 유지한다. 일반 flight_manager의 기본 옵션은 false다.


## 실물 치수 기반 추가 점검

408×408×245.5mm 치수와 USD 외형을 대조한 결과 및 `hull`/`--fine-inflation` 실험 옵션은 [기체 외형·팽창 점검](zav-footprint-audit.md)에 별도 기록한다. 이 옵션의 전체 dense 임무는 아직 미통과이며, 위 기존 비교 설정은 보존한다.
