# ZAV 센서와 FAST-LIVO 실행 도구

이 디렉토리의 소스를 저장소에서 직접 실행합니다. 별도 배포는 필요하지 않습니다.
원본 설명은 ZAV 저장소의 `simulator/README.md`, 변경 이유는 `simulator/CHANGES-SENSORS.ko.md`에 있습니다.

- `ros_env.sh`: Ubuntu Fast DDS / WSL Cyclone TCP 자동 선택, 설치 위치와 domain 유지.
- `run_mid360_rviz.sh`: 공통 센서 수신기 + RViz. 창을 닫으면 수신기도 종료합니다.
- `run_fastlivo.sh`, `fastlivo_sim.launch.py`: 원본 실센서 YAML + 시뮬레이터 입력 토픽 override.
- `cyclonedds_*.xml`, `ensure_ros_daemon.py`: WSL TCP 우회에만 사용합니다.
- 실행 코드는 `../runtime`; 과거 측정/실험은 `../diagnostics`이며 정상 실행에 필요하지 않습니다.

```bash
~/run_zav_sim.sh
~/run_mid360_rviz.sh
~/run_fastlivo.sh
```

RViz의 Fixed Frame은 `camera_init`이며 등록 점군, odom, path, RGB를 표시합니다.
LiDAR는 40채널 rotary 근사 모델이고 실제 Livox 비반복 패턴은 아닙니다.
표시된 Hz는 설정된 시뮬레이션 주기와 실제 처리 속도를 구분해서 해석해야 합니다.
