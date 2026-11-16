# SUPER 공식 dense / high-speed 벤치마크

이 워크스페이스에서 원본 SUPER 예제를 ROS 2 Humble로 실행한다. 공식 PCD, 목표 파일, 센서 설정, 플래너 YAML은 바꾸지 않았다. ROS 1 `roslaunch` 대신 아래 실행기를 사용한다.

```bash
cd /home/goat/zenith_ws
./scripts/run_super_benchmark.sh dense
```

시작 후 약 3초에 자동으로 `(0,-50,1.5)` → `(0,50,1.5)` → `(0,-50,1.5)`를 비행한다. 좌표계는 `world`, 단위는 m다. 원본 목표 파일의 도착 반경은 각 1m다. dense는 전체 지도와 FPV RViz 두 창이 열린다. 왕복 완료 후 창은 그대로 유지된다. `Ctrl+C`로 종료한다.

다음 예제는 앞 예제를 종료한 후 실행한다.

```bash
./scripts/run_super_benchmark.sh high_speed
```

high-speed는 전체 지도 RViz 한 창이 기본이다. `fpv:=true`를 붙이면 FPV도 연다. 같은 실험을 다시 하려면 종료 후 다시 실행한다. RViz만 끄려면 `rviz:=false fpv:=false`를 붙인다. LiDAR 렌더러는 이 경우에도 OpenGL/X 디스플레이가 필요하다.

이 실험에서는 `env -> zav_sim`, `run_fastlivo.sh`, `flight_manager`, `zav_super_replanner.launch.py`를 추가 실행하지 않는다. 공식 `perfect_drone_sim`이 LiDAR와 정확한 odometry를 만들고, 공식 `super_planner`가 직접 위치 명령을 보낸다. 별도 DDS domain 118을 사용한다. 기존 PX4 실행과 토픽을 섞지 않는다. 실행기는 같은 domain에서 두 벤치마크의 중복 실행도 차단한다. 기본 domain은 `SUPER_BENCHMARK_DOMAIN`으로 바꿀 수 있다.

## 원본과 같은 부분

| 항목 | dense | high_speed |
|---|---|---|
| 지도 | `random_map_2_26609.pcd` | `random_map_24_6635.pcd` |
| 시작 자세 | `(0,-50,1.5)`, yaw 1.5741 rad | 동일 |
| 목표 순서 | `(0,50,1.5)` → `(0,-50,1.5)` | 동일 |
| 속도 제한 | 8m/s | 25m/s |
| 가속도 제한 | 15m/s² | 20m/s² |
| 재계획 | 15Hz | 15Hz |
| 계획 horizon | 7m | 30m |
| 기체 반경 | 0.2m | 0.2m |
| LiDAR range / rate | 15m / 10Hz | 70m / 10Hz |
| ROG-Map resolution | 0.05m | 0.3m |

원본: [hku-mars/SUPER](https://github.com/hku-mars/SUPER/tree/2ad3419c127a617c6d7df6925e81a14175a9c096), 고정 커밋 `2ad3419c127a617c6d7df6925e81a14175a9c096`.

원본의 ROS 2 패키지 템플릿을 사용하고, Humble 빌드에 필요한 누락 의존성·`std::isnan`·하드코딩된 CMake 경로를 수정했다. ROS 2 launch의 중복 argument와 upstream high-speed 파일명 오타를 우회하는 launch를 제공한다. 최적화 알고리즘이나 속도/제약 값을 완화하지 않았다. 데이터와 YAML은 고정 커밋의 바이트와 일치하는지 준비 스크립트가 검사한다.

## 다시 빌드 / 자동 검증

현재 머신은 이미 빌드했다. 캐시가 없으면 실행기가 준비와 빌드를 자동 수행한다. 수동 재빌드는 다음과 같다.

```bash
python3 scripts/prepare_super_benchmarks.py --build
```

별도 `.cache/super_benchmark_ws`에서 6개 패키지를 빌드하므로 기존 ZAV의 `rog_map`과 충돌하지 않는다. ROS 2 Humble 개발 환경, PCL/Eigen/OpenCV/yaml-cpp, GLFW/GLEW/OpenGL, `mavros_msgs`, RViz 및 시스템 Python NumPy/SciPy가 필요하다. 캐시의 `provenance.json`에 원본 해시와 호환성 수정 목록이 기록된다.

실제 비행 odometry를 기록하고 두 목표의 순서, 왕복 도달, 속도와 점군 여유 거리를 독립 검사하려면:

```bash
./scripts/check_super_benchmark.sh dense --output /tmp/super-dense-check
./scripts/check_super_benchmark.sh high_speed --output /tmp/super-high-check
```

`--rviz`를 붙이면 검증 중 전체 지도도 표시한다. 각 검증은 종료 시 자신이 시작한 프로세스를 닫는다. `report.json`, `odometry.jsonl`, `launch.log`가 남는다. 검증할 때는 다른 native benchmark를 종료한다.

검증은 odometry 사이 선분을 최대 2cm 간격으로 보간해 원본의 유한 PCD 점과의 최근접 거리를 계산한다. 선분 전체 거리의 하한으로 표본 최소 거리에서 1cm를 빼고, 이것이 기체 반경 0.2m 이상인지 확인한다. 이는 원본 점군과 기록한 중심 경로에 대한 검사이며, 실제 기체의 메시 충돌 검사나 PX4 동역학 검증은 아니다.

## PX4 결과와의 구분

이 공식 예제의 `perfect_drone_sim`은 `PositionCommand`의 위치·속도를 그대로 상태에 대입한다. 모터 지연, 자세 제어 오차, FAST-LIVO 추정 오차를 모델링하지 않는다. 따라서 이 예제의 25m/s 왕복 성공이 ZAV/PX4에서 25m/s가 가능함을 뜻하지 않는다.

이전 ZAV/PX4 추종 시험은 별도의 1m/s 프로파일이다. 공식 숲을 Isaac의 충돌 메시로 옮기고, 센서 좌표계와 기체 크기를 맞춘 뒤, 저속부터 단계적으로 속도를 올리는 작업이 추가로 필요하다. 기존 flight manager의 0.5m 추종 오차 제한을 올려서 원본 속도를 억지로 통과시키지 않는다.

실행 및 검증 기록: [2026-11-05 결과](validation/super-benchmarks-20261105/README.md).
