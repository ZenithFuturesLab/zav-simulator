# Super-LIO 실행

Super-LIO ROS2를 기존 ZAV 시뮬레이터의 대체 LIO로 사용할 수 있다. 카메라를 융합하는 FAST-LIVO2와 달리 LiDAR와 IMU만 사용한다. 두 알고리즘을 동시에 실행하면 같은 odometry 토픽과 PX4 외부 위치 출력을 중복 발행하므로 실행 잠금을 공유한다.

```bash
# 기존 FAST-LIVO를 Ctrl+C로 종료한 뒤
./run_zav_sim.sh
# 다른 터미널
./run_superlio.sh
# 지도 표시가 필요하면
./run_superlio.sh rviz:=True
```

기존 `./run_fastlivo.sh`는 FAST-LIVO2를 실행한다. `run_superlio.sh`도 동일한 `px4_bridge`, `safety`, `safety_experiment`, `baro_param_connection` launch 인자를 지원한다. 물리/IMU는 200Hz, 렌더링은 20Hz를 유지한다. Super-LIO 실행기는 카메라 TCP 수신을 생략하지만 Isaac의 카메라 렌더링 설정은 변경하지 않는다.

| 용도 | 토픽 / 계약 |
|---|---|
| LiDAR 입력 | `/drone0/mid360/lidar` — `livox_ros_driver2/msg/CustomMsg`, 점별 offset ns |
| IMU 입력 | `/drone0/mid360/imu` — `sensor_msgs/msg/Imu`, m/s²·rad/s |
| IMU 예측 odometry | `/LIVO2/imu_propagate` |
| LiDAR 보정 odometry | `/aft_mapped_to_init` |
| 포인트클라우드 / 경로 | `/cloud_registered`, `/path` |
| Odometry 좌표계 | `camera_init` → `aft_mapped`; pose는 장착 IMU 원점, linear/angular twist는 IMU 좌표계 |
| PX4 / 안전 상태 | 기존 `lio_px4_bridge`와 `/zav/lio_safety/status` 사용 |

측정 시각을 보존하며 수신 시각으로 대체하지 않는다. 장착 IMU의 기울기와 레버암 제거는 기존 PX4 bridge가 수행한다. LiDAR→IMU 외부 보정은 시뮬레이터와 같은 `src/FAST-LIVO2/config/zenith.yaml`에서 실행 시 읽는다 (`PEGASUS_FASTLIVO_CONFIG_DIR`도 지원). upstream 기본 extrinsic의 Z 부호를 그대로 쓰지 않는다.

## 빌드

기존 ROS Humble/PCL/Livox 환경에 glog/TBB가 추가로 필요하다.

```bash
sudo apt install libgoogle-glog-dev libtbb-dev
./build_superlio.sh
```

이 환경에서는 해당 glog/gflags Ubuntu 패키지를 워크스페이스 `.local`에 풀어 사용했으며 시스템 패키지 설치는 변경하지 않았다. `build_superlio.sh`는 별도 dependency prefix를 지원하고 기본 병렬 컴파일 수를 2로 제한한다. PX4용 로컬 Fast-CDR 2.x와 ROS Humble의 Fast-CDR 1.x가 섞이지 않도록 빌드 경로를 분리한다.

## 소스와 호환 변경

- 원본: https://github.com/Liansheng-Wang/Super-LIO (`ros2`, `f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2`). `src/Super-LIO/UPSTREAM.md`, GPL-3.0 라이선스 포함.
- 맵·관측 갱신은 upstream을 유지하고 ROS wrapper에 opt-in `lio.output.fast_livo_compat`를 추가했다. ZAV 설정에서만 활성화한다. 후속 수동 조종 검증에서 IMU 예측의 지연 보정 뒤 재적분을 보완했다. [failsafe 분석](validation/keyboard-failsafe-20261029/README.md)을 참고한다.
- 세계 좌표계 속도를 IMU child 좌표계로 변환하고 기존 bridge가 요구하는 프레임 이름을 사용한다. 속도 대각 분산 0.04는 기존 FAST-LIVO와 같은 보수적 가정값이며 ESKF 공분산을 의미하지 않는다.
- Humble의 glog CMake 이름 및 명시적인 TBB 링크, extrinsic 회전 행렬의 row-major 해석, 비정상/빈 Livox 패킷 처리와 scan end의 최대 점 시각 계산을 보완했다.
- 기존 WSL2 Cyclone TCP relay·PX4 DDS gateway·barometer safety·실험 relay를 공통 실행 경로로 재사용한다.

## 검증 결과

200Hz 실제 SITL에서 이륙·호버·1m 이동 두 번·복귀·90도 회전·착륙·모터 해제까지 통과했다. 비행 중 safety latch는 없었다. 속도/프레임 단위 검사와 공통 실행기의 종료·재시작 검사도 통과했다. 기존 FAST-LIVO의 센서 입력·map update 실행도 다시 확인했다.

시험 범위, 수치, 중단된 계측 회차와 원인 정정은 [검증 기록](validation/super-lio-20261029/README.md)에 정리했다. 렌더 설정 20Hz를 실제 20FPS 확보로 해석하지 않는다.
