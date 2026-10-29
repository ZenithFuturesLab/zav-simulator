# Super-LIO 통합 검증 — 2026-10-29

Windows Isaac Sim 5.1 + WSL2 Ubuntu 22.04 / ROS Humble / PX4 SITL / Full Warehouse에서 검증했다. Super-LIO ROS2 commit `f89f48dc7aea6cfa262f18e4d03b319e04e0dbd2`에 ZAV 호환 ROS wrapper를 적용했다.

## 결과

- `build_superlio.sh -DBUILD_TESTING=ON`: basic / livox_ros_driver2 / super_lio 빌드 완료. Humble Fast-CDR 1.x와 PX4용 로컬 2.x 경로를 분리했다.
- C++ 호환 단위 검사: 90도 회전 시 세계→IMU 속도 변환, 장착 IMU 회전 왕복, frame/stamp/pose 보존 및 양의 속도 분산 계약 통과.
- 공통 실행기: backend 선택·카메라 relay 생략·FAST-LIVO/Super-LIO 공용 잠금 검사 통과. 실제 PTY Ctrl+C/HUP/TERM/중복 실행/종료 후 재시작 검사 통과.
- Super-LIO 초기화 → map init → LIO safety READY 확인.
- 물리/IMU **200Hz**, 렌더 설정 **20Hz** 유지.
- 실제 flight manager로 **2m 이륙 → 15초 호버링 → 1m 이동 두 번 → 복귀 → 90도 회전 → 10초 호버링 → 착륙·모터 해제** 완료.
- 이동·회전 통과 조건: 위치 오차 <0.2m, 추정 속도 <0.25m/s, 헤딩 오차 <10도를 시뮬레이션 시간 3초 연속 유지 (각 단계 제한 25초).
- 비행 중 safety latch 없음. 강제 Arm/Disarm 또는 safety 해제 없음.
- Isaac 내부 state에서 독립적으로 측정한 첫 호버링 마지막 약 8초의 X/Y/Z 최대−최소 범위: **0.87 / 0.83 / 0.50cm**. 최종 호버링은 **1.11 / 1.29 / 1.43cm**.
- IMU는 시뮬레이션 시간 기준 약 200Hz, 실제 수신 약 140.5Hz. 표본 프레임 중앙값 약 72ms. 렌더 설정 20Hz가 실제 시간 20FPS 달성을 뜻하지 않는다. FAST-LIVO와의 통제된 성능 비교는 수행하지 않았다.
- Super-LIO 종료 후 기존 FAST-LIVO를 `px4_bridge:=False safety:=False camera:=False`로 실행해 실제 센서 입력·LIO map update 정상 동작 확인 (비행 제어 출력 차단 상태의 실행 회귀 검사).

## 중단 회차와 계측 수정

초기 GUI 시험은 복귀 중 중단됐고, headless 시험과 다음 재시험도 중단됐다. 이를 성공으로 간주하지 않았다. 원인 추적 시 종료 직전 Kit의 `is_running=true`, `is_exiting=false`, stage 유효 상태를 확인했다. 임시 검증 코드의 Windows 파일 교체와 WSL reader 간 경합에서 `PermissionError`가 발생하는 것을 별도 부하 시험으로 재현했다. 해당 계측 코드가 예외 후 `finally`에서 SimulationApp을 닫아 정상 종료처럼 보이게 만들었다.

검증용 telemetry 교체 실패 시 다음 표본에서 재시도하도록 보완하고, 다른 예외는 종료 전에 traceback을 기록하도록 변경했다. 그 후 GUI 상태의 전체 임무 시험이 끝까지 통과했다. 이 계측 코드는 `.cache`에만 있으며, 제품의 `simulator/zav_sim.py`에는 포함하지 않는다. 중단 기록을 Isaac 자체의 런타임 결함이라고 확정하지 않는다.

## 재현 자료

간추린 수치는 [metrics.json](metrics.json)에 포함했다. 큰 원본 로그는 로컬 `.cache/super-lio-validation/`에 보존했다.

- `flight-final.jsonl`, `flight-final-console.log`, `result-final.json`: 최종 통과 회차.
- `super-lio-final.log`: mapper·bridge 상태와 정상 종료 기록.
- `flight*.jsonl`, `result-*-interrupted.json`: 중단 회차.
- `exit-state.json`, `rename-probe-result.txt`: 계측 종료 상태 및 공유 파일 경합 재현.
- `.cache/super-lio-build-final.log`, `.cache/super-lio-tests-final.log`, `.cache/super-lio-supervisor-test-final.log`: 빌드·회귀 검사 기록.

실기체·고속 기동·장시간 운용은 이 시험 범위에 포함하지 않는다. Super-LIO는 카메라 융합을 제공하지 않으며, 여기서 맞춘 것은 기존 LiDAR/IMU·odometry·PX4 bridge·safety 인터페이스다.
