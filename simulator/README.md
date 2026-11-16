# 시뮬레이터 소스

이 폴더 자체가 실행 원본입니다. `zav_sim.py`와 `pegasus/`의 코드를 수정하고
루트의 `./run_zav_sim.sh`로 재실행하세요. 복사·배포·Python 빌드가 필요하지 않습니다.

- `zav_sim.py`: 주기, 센서 설정, 장면, 메인 루프
- `pegasus/simulator/logic/vehicles/multirotors/zav_v1.py`: 기체 설정
- `pegasus/simulator/logic/sensors/`: IMU·캘리브레이션
- `pegasus/simulator/logic/graphical_sensors/`: LiDAR·카메라
- `pegasus/simulator/logic/backends/`: PX4·ROS·TCP 통신
- `pegasus/simulator/assets/`: 기체·센서 USD
- `tools/`: ROS 센서 수신·FAST-LIVO·RViz 실행
- `scripts/`: Isaac·PX4·tmux 대시보드
- `config/pegasus.yaml`: 공통 초기 좌표 등
- `px4/`: PX4 빌드에 등록하는 airframe
- `environment.sh`, `ros_setup.sh`: 컴퓨터 설정·ROS 환경

Pegasus 기반 소스를 이 저장소에 포함했으며 출처 커밋은 `pegasus/UPSTREAM.json`,
라이선스는 `pegasus/LICENSE`에 보관합니다. 외부 Pegasus 설치를 수정하지 않습니다.
최초 환경 설치와 C++ 빌드는 [루트 README](../README.md)를 참고하세요.
