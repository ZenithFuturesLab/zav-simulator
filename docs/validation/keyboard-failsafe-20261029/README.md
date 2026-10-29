# 수동 조종 중 LIO failsafe 분석 — 2026-10-29

## 사용자 로그에서 확인한 발동·종료 경로

최근 Super-LIO 실행의 `python3_816727_1793265975926.log`에서 18:27:22(KST)에 `LIO SAFETY FAILED: ['attitude_residual']`가 기록됐다. 약 0.255초 뒤 기압계 융합·유효 수직 상태를 확인하고 NAV_LAND를 요청했으며 PX4 ACK=0(수락)을 받았다.

동일 launch의 실제 프로세스 종료는 18:27:51의 `user interrupted with ctrl-c (SIGINT)` 뒤 정상 종료로 기록됐다. 즉 자세 점프 감지에 의한 자동 착륙과 프로세스 종료는 별개다. 해당 로그에서 mapper crash나 E/Q 키에 의한 종료가 확인된 것은 아니다.

## 실제 재현

물리/IMU 200Hz·렌더 설정 20Hz, Super-LIO와 기존 flight manager/bridge/safety를 실행했다. desktop 전체에 키를 주입하지 않고 실제 Win32MultiKeyReader의 입력 pipe/parser에 키 마스크를 넣어 같은 수동 조종 경로를 구동했다. Windows GetAsyncKeyState 자체의 지연까지 재현한 시험은 아니다. T로 이륙하고 전후·정지·좌우 회전·전진+회전 입력을 실행했다. LIO 원본 odometry, 실제 IMU, Isaac state, 키 이벤트, safety 상태를 함께 기록했다.

- 긴 전진 입력: 명령 속도 12m/s, 실제 속도 약 9.9m/s에 도달한 뒤 기체 각속도·속도에 급격한 변화가 나타났다. LIO 자세는 5ms 사이 **56.48°** 바뀌어 허용치 **21.15°**를 초과했다. 위치 잔차는 약 0.149m로 허용치 이내였다. `attitude_residual`로 착륙·disarm 완료.
- 이 고속 시험에서는 실제 기체도 급격히 회전했다. 창고 구조물 접촉 또는 운동 불안정의 영향이 의심되지만 접촉 이벤트를 계측하지 않았으므로 충돌을 확정하지 않는다. 이 실패를 전부 알고리즘 또는 키보드 오류로 간주하지 않았다.
- 짧은 조종 입력: 수정 전에도 전체 조종·착륙을 통과했다. 키 입력 자체가 항상 failsafe를 발생시키는 것은 아니다.

## 확인하고 수정한 문제

### Super-LIO IMU 예측의 시간 정합

원본 `ESKF::Update()`는 LiDAR 보정 시 최신 IMU 예측 상태를 과거 scan 시각의 상태로 되돌리지만, 그 사이 IMU 기록을 다시 적분하지 않았다. 다음 IMU 한 쌍만으로 전체 지연 구간을 적분해 각속도가 변하는 구간에서 인공적인 자세 변화가 발생할 수 있었다.

관측 잔차·관측 정보가 모두 0인 독립 시험에서, 실제 다음 IMU 적분량은 약 1.17°인데 출력이 약 13.21° 변하는 현상을 재현했다. 수정은 scan 보정 뒤 실제 IMU history를 보정 시각부터 현재까지 재적분하는 방식이다. 보정 경계 IMU는 보간하고 기록은 최대 2초로 제한한다. 출력 시각을 바꾸거나 자세를 임의로 smoothing하지 않는다.

동일한 짧은 조종 패턴의 실제 SITL 비교:

| 조건 | 연속 IMU odometry 간 최대 자세 변화 | 결과 |
|---|---:|---|
| 원본 | 13.66° | 조종·착륙 통과 |
| 재적분 수정 후 | 2.37° | 조종·착륙·disarm 통과 |

이 비교에서는 두 회차 모두 failsafe가 없었다. 따라서 개선 수치를 모든 과거 실패의 단독 원인이 확정됐다는 의미로 해석하지 않는다. 다만 시간 정합 버그와 출력 불연속 감소는 독립 수치 시험 및 실제 조종 비교로 확인했다.

### 키 입력 연결 단절

기존 키 리더는 Windows 입력 pipe가 끊기거나 더 이상 갱신되지 않아도 마지막 눌림 비트마스크를 유지했다. 이 때문에 키를 놓았는데도 이동 명령이 계속될 수 있었다. **250ms 갱신 중단 / EOF / 읽기 오류 시 키 상태를 0으로 해제**하도록 수정했다. 키 입력 단절 자체로 자동 착륙을 발동시키는 변경은 아니다. 실제 사용자 실패 당시 pipe가 끊겼다는 증거는 없으며 별도로 확인한 결함이다.

현재 Windows GetAsyncKeyState는 전역 키 상태를 읽으므로 다른 창의 WASD·방향키·Space 입력도 반영될 수 있다. 이번 변경에서 focus 제한이나 키 배치는 바꾸지 않았다.

### 재발 시 진단

최초 safety latch 시 `LIO jump evidence` 로그에 측정 간격, 잔차/허용치, 이전·현재 quaternion, 위치를 남기도록 했다. safety 임계값·기압계 전환·착륙 절차는 변경하지 않았다. 최대 수동 속도 12m/s 설정도 유지했다. 창고 내 고속 이동과 실제 충돌에 대한 안전 동작까지 제거한 것이 아니다.

## 검사 및 상태

- Super-LIO 빌드 및 C++ odometry/재적분 검사 통과 (`colcon test-result`: 5 tests, 0 errors/failures).
- LIO safety 정책 17개 검사 통과.
- 키 입력 timeout/EOF/읽기 오류 및 최신 마스크 검사 통과.
- PX4 유효 속도 이륙 검사·소스 배치 검사·`git diff --check` 통과.
- 최종 실제 조종 시험에서 safety latch 없음, 착륙 및 모터 해제 확인.
- 임시 Isaac 계측은 제거했고 기본 물리200Hz·렌더20Hz를 유지했다. 테스트 프로세스는 종료했다.

큰 원본 자료는 `.cache/keyboard-failsafe-20261029/`에 보존했다. `manual-run1.jsonl`/`fast-collision-jump-events.json`은 고속 재현, `manual-short.jsonl`/`short-baseline-raw-odom.jsonl`은 수정 전 짧은 조종, `manual-fixed2.jsonl`/`manual-fixed2.odom.jsonl`/`result-fixed.json`은 수정 후 비교다. 초기 preflight 계측 재시작 회차는 성능 판정에 포함하지 않았다.
