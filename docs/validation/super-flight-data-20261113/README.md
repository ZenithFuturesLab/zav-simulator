# SUPER 주행 데이터 기록·분석 검증 — 2026-11-13

Isaac Sim + RViz2 GUI에서 high_speed / super_lio 왕복 후 착륙을 확인했다. 기존 주행 파라미터를 변경하지 않고 passive recorder를 추가했다. 자동 기록 원본과 브라우저 보고서는 `log/super_flights/20261113-021158-553390/`에 있다.

[주행 그래프](motion.png), [최댓값·시각·계산 조건](summary.json), [실행 설정](metadata.json).

실제 body 기준 TRACKING 최대값: 속도 5.831648m/s, 가속도 5.406376m/s², 저크 36.533902m/s³. 가속도·저크는 속도 벡터의 약 0.21초 Savitzky–Golay 평활 미분이며 무필터 순간 접촉 최대값이 아니다. 최종 controller 상태는 LANDED였다. 검증 하네스가 LANDED 직후 navigation을 종료하여 이 기록의 종료 사유는 interrupted이며, launcher 종료 후 후처리가 정상 완료됐다. 이후 recorder는 LANDED 직후 종료된 경우에도 metadata 완료 상태를 LANDED로 기록하도록 보완했다.

검증:

- `python3 simulator/tests/test_super_flight_metrics.py`: 4개 통과. 벡터 미분, 일정 속력 원운동의 구심 가속도/저크, 회전하는 mounted IMU의 lever-arm 보정, 수신 공백·시계 역행·NaN 구간 분리를 해석적인 운동과 비교했다.
- 별도 ROS domain 177에서 합성 odometry만 발행해 truth/LIO 구독, LANDED 자동 후처리, LANDED 직후 SIGINT 시 원본 flush 및 offline 후처리를 검증했다. 합성 데이터는 실제 비행 결과와 분리하여 `/tmp/super-telemetry-test-*`에 저장했다. PX4 제어 토픽은 발행하지 않았다.
- 실제 GUI 비행에서는 `run_super.sh`의 telemetry_dir 전달 → ROS launch recorder → 원본 저장 → 후처리 → HTML/PNG/CSV/JSON 생성을 확인했다. 브라우저 열기 helper도 확인했다.
- Python 구문 검사 통과, 설치 launch와 소스 launch가 일치하며 현재 planner 설정이 metadata의 실행 설정과 의미상 동일함을 확인했다.

사용법: [SUPER 주행 데이터 보기](../../super-flight-data.md).
