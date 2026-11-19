# EPIC ROS2

[EPIC](https://github.com/Robotics-STAR-Lab/EPIC)의 ROS2 포트다. 원본의 frontier 군집, viewpoint sampling, LKH TSP, bubble A* 및 MINCO 로컬 플래너 구조를 바탕으로 한다. ROS2 메시지와 호환 계층을 이 패키지에 포함한다. 라이선스는 `LICENSE`를 참조한다.

이 디렉터리에는 EPIC 알고리즘, ROS2 연결, 설정과 회귀 테스트를 포함한다. SUPER, LIO 추정기, PX4 제어기, Isaac Sim 장면과 워크스페이스 실행 스크립트는 별도 구성이다.

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

입력 토픽은 `external_odometry_topic`, `external_cloud_topic`으로 지정한다. 위치와 포인트는 같은 world 좌표를 사용하고, odom 선속도도 world 기준이어야 한다. Ground truth는 탐색·제어 입력으로 사용하지 않는다. `exploration.forward_viewpoint_yaw=true`이면 관측 yaw를 경로 quaternion에 포함하며, 수신 측이 위치와 방향을 함께 전달해야 한다.

이 launch는 기존 ZAV의 `fast_livo` 보정 설정을 읽는다. LIO 실행, 센서·시계 공급, 이륙 trigger, 외부 궤적 생성과 PX4 제어는 별도로 연결해야 한다. 워크스페이스의 `run_epic*.sh`나 SUPER 연동은 이 패키지의 일부가 아니다.

LKH 임시 문제·해 파일은 launch가 `/tmp`에 생성하고 종료 시 정리한다. 기존 표시 토픽을 보는 명령은 `ros2 launch epic_ros2 rviz.launch.py`다.

## 현재 검증 범위와 남은 문제

2026-11-19 Isaac Sim headless / Super-LIO / SUPER 연동 시험에서 3층 건물은 실제 방 18/18 진입과 FINISH를 확인했다(515.75 시뮬레이션 초). 동일 설정의 2층 건물은 6/12 진입 후 계단 중간에서 정체했다. 임의의 건물에 대해 검증 완료된 버전은 아니다.

남은 문제는 계단 하부의 비행 불가능한 관측 목표를 반복 선택하는 경우와, 경로 재발행 때 정체 감시 타이머가 초기화되는 처리다. 외부 플래너의 관측 자유공간 판정과 전역 후보 연결을 일치시키고, 동일 실패 목표의 정체 시간을 유지하는 수정이 필요하다. 이 미해결 처리는 시험 결과를 숨기지 않기 위해 현재 소스에 그대로 남아 있다.
