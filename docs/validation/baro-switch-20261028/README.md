# 기압계 융합을 발산 시에만 활성화하는 SITL 검증

실제 Isaac Sim Full Warehouse, FAST-LIVO2, PX4 SITL과 flight manager로 약 2 m 이륙·호버·착륙을 수행했습니다.
기압계 파라미터 응답과 실제 융합 상태를 mock하지 않았습니다. 점프 오류만 감시용 pose에 주입합니다.

| 시험 | 정상 비행 중 | 점프 후 실제 융합 확인 시간 | 최종 결과 |
|---|---|---:|---|
| 일반 실행 모드 정상 착륙 | 파라미터 0 / 융합 false 유지 | 전환 없음 | 지면 복귀 + landed + disarmed |
| 위치 X +3 m | 파라미터 0 / 융합 false 확인 | 0.608초 | 파라미터 1 + 융합 true → 착륙·disarm |
| yaw +90° | 파라미터 0 / 융합 false 확인 | 0.706초 | 파라미터 1 + 융합 true → 착륙·disarm |

모든 시험이 PASS입니다. 점프 시험은 `baro_transition_confirmed=True`,
`baro_transition_timed_out=False`, `landing_complete=True`까지 확인했습니다.
기압계 활성화 후에도 GPS=0, EV 설정=15, 설정상 높이 기준=EV(3)는 유지했습니다.
EV 차단 후 PX4가 내부 높이 기준을 fallback하고, 수평 위치 상실 시 DESCEND로 내려옵니다.
강제 arm/강제 disarm/kill로 결과를 통과시키지 않았습니다.

`summary.json`과 세 JSONL에 실제 simulator ground truth, PX4 모드·유효성·착륙 감지,
기압계 값과 융합 여부, ACK 및 전환 시간 증거를 보존했습니다.
`px4-console.txt`에는 일반 실행 종료 후 기압계 파라미터 0, GPS=0, HGT_REF=3 실효값을 기록합니다.

29개 자동 테스트도 통과했습니다. 점프 판정, ROS 명령/착륙 확인,
파라미터 ACK만으로 전환하지 않는지, 활성화 timeout을 성공으로 표시하지 않는지,
PX4 int32 bytewise 인코딩, 다른 기체·파라미터·타입 응답 배제,
오래된 응답 배제 및 비행 중 기압계 비활성화 쓰기 금지를 포함합니다.

재현은 [실험 안내](../../lio-safety.md)를 따릅니다.
SITL의 정상적인 기압계에서 검증한 결과이며, 실제 실내 압력 변화나 기압계 동시 고장을 검증하지 않았습니다.
본 구현은 점프 판정만 사용하므로, 점프 없는 FAST-LIVO 출력 중단은 기압계 전환 트리거가 아닙니다.
