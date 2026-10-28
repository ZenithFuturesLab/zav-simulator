# 실제 SITL 자동 착륙 검증

이 문서는 기압계를 상시 융합했던 이전 방식의 기록입니다. 현재의 발산 시 활성화 방식은
[조건부 기압계 전환 검증](../baro-switch-20261028/README.md)을 참고하세요.

Isaac Sim의 Full Warehouse, FAST-LIVO2, 실제 PX4 SITL과 flight manager를 실행하여
약 2 m 이륙·호버 후 착륙을 검증했습니다. mock 메시지로 착륙 성공을 대신하지 않았습니다.
점프 실험의 오류는 감시용 odometry에 주입하고, 실제 물리 기체는 PX4 제어로 착륙합니다.

## 재현된 원인과 수정

수정 전: 위치 점프 → 외부 odometry 차단 → 높이/수평 위치 무효화 → 비행 종료 상태(13).
실제 기체가 지면에 도달해도 landed/disarm 완료로 이어지지 않았습니다.
`before-fix-transitions.jsonl`에 해당 상태 전이를 남겼습니다.

수정 후: `EKF2_BARO_CTRL=1`로 기압계 높이 융합을 유지합니다.
GPS=0, 외부 추정 융합=15, 정상 높이 기준=EV(3)는 유지합니다.
실효 파라미터와 PX4의 착륙·자동 disarm 메시지는 `px4-console.txt`에 있습니다.
점프 판정 기준은 운동 연속성만 사용하며 추가 실패 판정은 넣지 않았습니다.

| 시험 | 결과 | 착륙 시작부터 완료까지 약 실제 시간 | 모드 |
|---|---|---:|---|
| 일반 실행 모드의 정상 착륙 | 지면 복귀 + landed + disarmed | 7.10초 | AUTO_LAND |
| 위치 X +3 m 점프 | latch + 지면 복귀 + landed + disarmed + 감시기 완료 확인 | 6.78초 | AUTO_LAND → DESCEND |
| yaw +90° 점프 | latch + 지면 복귀 + landed + disarmed + 감시기 완료 확인 | 7.62초 | AUTO_LAND → DESCEND |

각 시험의 JSONL과 `summary.json`에 simulator ground truth, 모드, 추정 유효성,
착륙 감지와 안전 상태를 기록했습니다. 1초 간격 로그에서 계산한 시간이므로 약 1초의 오차가 있습니다.
강제 disarm/kill로 성공 처리하지 않았습니다. 초기 arm 요청은 PX4의 일반 arming check를 통과해야 하며,
검증 스크립트는 최대 세 번 일반 arm을 요청합니다.

점프 시험에서 수평 위치가 무효화된 뒤에도 높이는 계속 유효했고,
PX4의 DESCEND가 수평 위치 유지 없이 하강을 마쳤습니다.
지면 접촉 후 자동 disarm이 확인됐고, 감시기의 `landing_complete=True`도 확인했습니다.
일부 최종 레코드의 OFFBOARD는 **disarm 후** 상태이며 공중에서 offboard를 재개한 것이 아닙니다.

추가로 중간 재시작 후 일반 착륙을 시도한 별도 시험에서는 실제 LIO 결과에 약 0.65 m의 위치 불연속이
발생했습니다. 이 경우도 착륙·disarm은 완료됐지만 정상 비행 통과로 세지 않았습니다.
위 표의 정상 착륙은 전체 재시작 후 오류 없이 별도로 통과한 기록입니다.

재현 명령은 [실험 안내](../../lio-safety.md)의 `check_autoland.py` 항목에 있습니다.
본 결과는 이 SITL 프로파일의 약 2 m 시험입니다. 실제 기압계 이상, 바람, 높은 고도,
장시간 관성 추정과 장애물 환경까지 검증한 결과는 아닙니다. DESCEND는 횡방향 표류를 보장하지 않습니다.
