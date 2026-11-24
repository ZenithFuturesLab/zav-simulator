# EPIC ROS2

[EPIC](https://github.com/Robotics-STAR-Lab/EPIC)의 ROS2 포트다. 원본의 frontier 군집, viewpoint sampling, LKH TSP, bubble A* 및 MINCO 로컬 플래너 구조를 바탕으로 한다. ROS2 메시지와 호환 계층을 이 패키지에 포함한다. 라이선스는 `LICENSE`를 참조한다.

이 디렉터리에는 EPIC 알고리즘, ROS2 연결, 설정과 회귀 테스트를 포함한다. SUPER, LIO 추정기, PX4 제어기, Isaac Sim 장면과 워크스페이스 실행 스크립트는 별도 구성이다.

Isaac Sim에서 비행하려면 워크스페이스 루트에서 `./epic start clinic`을 실행한다. 실행 중 `return`을 입력하면 SUPER가 시작점으로 복귀한다. [세 맵 실행과 조작 안내](../../README.md#epic-탐사-빠른-시작)

## 빌드와 회귀 테스트

ROS2 Humble 작업 공간에서 PCL, Eigen3, OpenCV, OpenMP와 `package.xml`의 ROS 의존성을 준비한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select epic_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select epic_ros2
colcon test-result --verbose
```

테스트는 MINCO 비용 적분, 경로 연결, frontier 군집/관측 지지, 표면 가시성을 검사한다. 비행 완료를 보장하는 테스트는 아니다.

## 코드 흐름

- `src/global_planner/lidar_map`: 측정 포인트맵과 ikd-tree.
- `src/global_planner/frontier_manager`: frontier 군집, 관측 상태와 viewpoint sampling.
- `src/global_planner/pointcloud_topo`: bubble topology와 경로 연결.
- `src/global_planner/exploration_manager`: LKH 방문 순서, 전역 경로 및 탐색 상태기계.
- `src/local_planner`: 원본 EPIC의 경로 탐색과 MINCO 로컬 궤적.
- `include/epic_ros2`, `msg`: ROS2 호환 계층과 메시지.

## 외부 로컬 플래너 연결

기존 ZAV 환경의 `isaac_epic.launch.py external_local_planner:=true`는 EPIC의 로컬 MINCO와 `traj_server`를 실행하지 않고 `/mission/exploration_path` (`nav_msgs/Path`, world)를 발행한다. 기본값 `false`는 EPIC 로컬 계획을 사용한다.

Isaac 탐색 설정에서 `global_planning.return_to_start`의 기본값은 `false`다. EPIC이 남은 탐색 목표가 없음을 확인하면 `/mission/exploration_complete`를 발행한다. 워크스페이스의 SUPER 연동 FSM은 이 신호와 사용자 복귀 명령을 받아 이륙 위치로 복귀하고 호버한다.

입력 토픽은 `external_odometry_topic`, `external_cloud_topic`으로 지정한다. 위치와 포인트는 같은 world 좌표를 사용하고, odom 선속도도 world 기준이어야 한다. Ground truth는 탐색·제어 입력으로 사용하지 않는다. `exploration.forward_viewpoint_yaw=true`이면 관측 yaw를 경로 quaternion에 포함하며, 수신 측이 위치와 방향을 함께 전달해야 한다.

이 launch는 기존 ZAV의 `fast_livo` 보정 설정을 읽는다. LIO 실행, 센서·시계 공급, 이륙 trigger, 외부 궤적 생성과 PX4 제어는 별도로 연결해야 한다. 워크스페이스의 `run_epic*.sh`나 SUPER 연동은 이 패키지의 일부가 아니다.

LKH 임시 문제·해 파일은 launch가 `/tmp`에 생성하고 종료 시 정리한다. 기존 표시 토픽을 보는 명령은 `ros2 launch epic_ros2 rviz.launch.py`다.

## 탐색 목표 선택

관측한 frontier를 군집화하고 각 군집에서 가시성이 있는 뷰포인트를 선택한다. EPIC은 연결 가능한 후보의 방문 순서를 LKH로 계산하고 SUPER에 전역 경로를 전달한다. RViz의 Sampled viewpoints는 후보 선별 전에 샘플링한 위치를 표시한다.
