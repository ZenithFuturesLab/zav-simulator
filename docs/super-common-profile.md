# SUPER 공통 Super-LIO 설정과 검증 순서

`src/zav_super/config/common_super_lio.yaml` 하나에서 SUPER와 OMMPC의 설정을 읽는다. 기존 high_speed/dense YAML과 기존 명령의 기본 동작은 보존한다. 새 설정은 검증 중인 출발점이며 최적 설정으로 채택한 값이 아니다. SUPER 알고리즘, 환경별 전환 로직, Super-LIO는 변경하지 않는다.

```bash
./run_super_sim.sh high_speed
# 다른 터미널: Isaac Sim GUI + RViz2 사용
./run_super.sh --scenario high_speed --backend super_lio --profile common --execute
```

시뮬레이터와 navigation을 종료한 뒤 `dense`로 양쪽 명령의 scenario만 바꾸면 같은 설정으로 시험한다. `--profile legacy` 또는 profile 생략 시 기존 시나리오별 파일을 사용한다. 다른 LIO backend는 이번 공통 프로파일의 대상에서 제외한다.

## 현재 공통 실험 기준 (2026-11-15 후속 GUI 검증)

현재 common 기본 파일은 `radius050-time5e7` 후보와 바이트까지 같다. high_speed 3회·dense 3회 모두 원래 LIO 경유점 0.8m·정지 0.7m/s 조건으로 정상 완주·착륙했고, native 장애물/미분류 외부 접촉 보고와 계측 오류·유실은 0이었다. 전체 기록 CAD 여유 하한은 최소 0.139m였다. 이는 6회 측정 범위의 공통 실험 기준이며 최고속도 최적값이나 모든 환경의 안전 보증은 아니다. nominal max_vel=20m/s 도달도 확인하지 못했다.

이전 20Hz common 파일은 `common_super_lio_20hz_reference.yaml`로 보존했다. 기존 시나리오별 YAML과 legacy 실행은 그대로다. 새 common은 nominal control_period/step_T=0.02/0.02, Q_pos_xy=600, robot_r=0.50, exp opt_accuracy=1e-4, exp penna_t=5e7이다. 나머지 exp/backup 패널티도 하나의 파일에 고정하며 환경별 전환을 추가하지 않았다. 실제 명령 관측은 nominal 50Hz와 다르므로 실행별 observed_commands를 본다. 검증 시 GUI 물리 200Hz·렌더 20Hz와 선택적 contact observer를 사용했다.

상세한 반복 결과·한계·재현 방법은 [접촉/추종 진단](super-contact-tracking.md), `log/super_flights/contact-tracking-20261115/radius050-time5e7-qualification.json`, `comparison.md`와 `radius050-time5e7-motion-repeats.png`에 있다. 이후 goal_vel_en 비교는 별도 실험 파일로 수행하며 이 기본 파일에 바로 반영하지 않는다. 아래 초기 20Hz·100Hz 기록은 당시 실험의 이력이다.

## 확인된 조건과 아직 확인되지 않은 값

저자는 [실기 배포 설명](https://github.com/hku-mars/SUPER/issues/5)에서 MPC 100Hz, 제어 상태 100Hz 및 IMU propagation을 이용한 odometry 최대 200Hz, Mid360 cloud 30Hz를 설명한다. 이 정보가 MPC 예측 discretization이 0.01초임을 의미하지는 않는다. 공개된 설명만으로 실기 MPC 가중치와 전체 YAML을 복원할 수 없다.

초기 공통 파일의 `control_period: 0.01`은 제어 갱신 목표였다. 후속 시험에서 이 주기와 odom 도착 시 갱신하는 후보가 물리적으로 불안정해 채택하지 않았고, 당시 파일은 `control_period: 0.05`로 복원했다. 후속 접촉/추종 반복 검증을 거친 현재 common은 `control_period: 0.02`, `step_T: 0.02`이다. odom-triggered 갱신 코드도 되돌렸다. 현재 adapter의 wall timer는 10ms이며 실제 갱신율은 ROS callback/solver/GUI 부하에 따라 달라지므로 실행 로그의 feedback step으로 평가한다. 당시 `step_T: 0.05`는 기존 로컬 예측 간격을 유지했다. 기존 프로파일은 control_period를 생략하면 기존처럼 step_T로 갱신을 gate한다.

그 외 플래너 값은 사용자의 현재 high_speed_super_lio 설정을 그대로 출발점으로 복사했다. 기체 질량, 반경, 추력 보정과 제어 가중치는 이 시스템의 값이며 논문 기체의 실측값으로 표기하지 않는다. 센서 설정도 확인 없이 변경하지 않았다.

## 단계별 계획

1. 공통 파일 로딩, 기존 실행 호환성, 빌드, 실제 제어 갱신 주기를 확인한다.
2. high_speed와 dense 각각 fresh Isaac GUI + RViz2로 초기 시험한다. 같은 공통 파일을 사용하고 완주/착륙, 실제/명령 속도, acc/jerk, odom 연속성, 위치 오차, 충돌 여유, A* 시간 초과, heartbeat, solver 시간을 기록한다. 초기 단발 시험은 신뢰성 인증이 아니다.
3. 길고 열린 직선에서 계획 명령이 속도 상한에 도달하는지와 실제 추종을 분리한다. 그 뒤 공통 후보만 조정하며 두 기존 장면을 매번 회귀 평가한다. 위치 오차는 보고하고 완주·충돌 여유를 우선한다.
4. 개방→밀집→개방이 연속된 맵을 별도 자산으로 만든다. 긴 가속 구간, 충분한 제동 구간과 가려진 전환 구간을 포함하고 기존 맵은 보존한다. 기존 SUPER가 생성하는 속도 변화만 평가한다.
5. 최종 후보는 새 GUI 세션에서 반복 검증한다. 전체 소요시간과 최소 여유를 비교하고, 속도 상한 달성률은 충분히 열린 구간에서 별도로 계산한다. 밀집 구간의 낮은 속도를 일괄 탈락 기준으로 삼지 않는다.

원본 로그는 `log/super_flights/common-baseline/`에 보관한다. 새 시험에서 실패하면 해당 실패를 포함해 다음 단계 진행 조건을 재검토한다. 미검증 상태에서 20m/s 비행 또는 두 장면의 안전 완주를 보장하지 않는다.

## 초기 GUI 시험에서 확인한 제약

2026-11-14 high_speed 첫 시험은 첫 경유점 도달 후 복귀 계획 실패와 heartbeat 만료로 미완주했다. 제어 로그 feedback step은 10–30ms이므로 100Hz 달성을 확인한 상태가 아니다. simulator는 PHYSICS_HZ=200 / RENDER_HZ=20이며 LiDAR acquisition이 render에 연결되어 저자의 cloud 30Hz와도 다르다. 이번 변경에서는 센서·물리·렌더 설정은 유지했다. 원본 로그와 실패를 보존하며 이 주기 차이를 다음 검증 대상으로 둔다.

첫 공통 dense 시험은 출발 최적화 실패 275회로 TRACKING에 진입하지 못했다. READY에서 50초 이상 기다린 후 정상 착륙을 요청했다. 착륙은 완료했지만 임무 완주는 아니다. 두 시험의 설정 SHA256은 동일하며 장면별 evaluation.json을 보존했다. 다음 단계의 우선순위는 실제 제어 주기 변동 원인 확인 → dense 출발 최적화 실패 분리 → 열린 직선 속도 달성 확인 → 공통 후보의 두 장면 반복 평가이다. 혼합 맵 제작은 그 이후로 둔다.

## 후속 후보 비교 — 2026-11-14

모든 수치는 단발 GUI 실행 결과다. 반복 신뢰성 또는 최적값을 입증하지 않는다. 당시 공통 YAML은 실험 후보 `margin048-inflate4`였으며, 두 환경에서 안전 완주한 설정으로 채택한 파일이 아니다. 기존 시나리오별 YAML은 SHA256으로 변경되지 않았음을 확인했다.

| 후보 | high_speed | dense |
|---|---|---|
| 탐색 위치 패널티 10배 | 임무 완료, 실제 peak 10.90m/s; 첫 실제 목표 접근 1.39m | 출발 실패 |
| 탐색 위치 제약 표현 2 | 임무 완료, peak 12.15m/s; 첫 실제 목표 접근 1.85m | 약 87m 진행 후 heartbeat 만료, 여유 하한 −0.051m |
| 위 후보 + 계획 거리 15m/전진 거리 7.5m | 임무 완료, peak 8.85m/s; 복귀 실제 목표 접근 5.08m | 정상 왕복·착륙, 주행 107.84s, peak 5.94m/s, 여유 하한 −0.041m |
| 위 후보의 100Hz 갱신 시험 | 이륙 중 물리 상태 발산, 중단 | 출발 직후 보호 동작, 중단 |
| 반경 0.48m, 기존 팽창 | 첫 경유점 부근 heartbeat 만료 | 안전 통로 생성 실패로 출발 불가 |
| 반경 0.48m + 팽창 4단계 | 정상 왕복·착륙, 주행 32.31s, peak 9.13m/s, 여유 하한 1.111m | 복귀 중 210s 시험 제한으로 중단, peak 4.32m/s, 여유 하한 0.027m |

마지막 high_speed의 실제 경유점 접근 거리는 0.379/0.400m, LIO 오차는 max 0.649m / RMS 0.333m, odom stamp 평균 200Hz / 최대 간격 5ms였다. 마지막 dense는 LIO max 1.440m / RMS 0.746m, odom stamp 평균 약 187Hz / 최대 간격 30ms였다. 위치 오차를 임의의 0.5m 기준으로 탈락시키지 않았다. 물리 목표 접근과 충돌 여유는 별도로 평가했다.

여유 하한은 실제 body 반경 0.33m, PCD 재구성 상한 0.086603m와 경로 표본 여유 0.005m를 뺀 보수적 값이다. 음수는 접촉을 직접 검출했다는 뜻이 아니라 무접촉 여유를 입증하지 못했다는 뜻이다. 정상 착륙만으로 임무 성공으로 집계하지 않으며, 목표 통과 뒤 요청한 착륙과 출발 실패 뒤 요청한 착륙을 구분한다. dense 210s 제한은 보호 동작 실패와 다르게 시간 제한으로 기록했다.

첫 `diagnostic/dense`는 이전 준비 로그를 읽어 너무 일찍 시작했을 가능성을 발견해 비교에서 제외했다. 이후 runner는 시작 이후 생성된 준비 로그인지 확인한다. 정상 시작으로 재시험한 100Hz 후보도 불안정했으므로 이 실패는 별도 보존했다. 이 시험의 high_speed는 크게 발산한 물리 위치도 확인됐으며, 이후 runner에는 물리 장면 범위 이탈 중단 조건을 추가했다.

원본·로드한 YAML·GUI 화면·실제 vel/acc/jerk 그래프·JSON 분석은 `log/super_flights/common-followup/`에 있으며 `comparison.json`이 후보 비교의 진입점이다. 상세 motion 수치는 각 실행의 `telemetry/report.html`에서 확인한다.

다음 단계는 100Hz가 논문과 같다는 이유만으로 적용하는 대신 제어기의 예측 간격·입력 유지·실제 기체 응답을 먼저 검증하고, dense에서는 통로 중심 유도 비용(penna_attract)과 실제 추종 오차를 분리해 비교하는 것이다. 열린 직선 속도 시험과 혼합 맵은 아직 수행하지 않았다. 두 환경 공통 안전 완주 후보가 나오기 전에는 최적 설정으로 발표하지 않는다.

## 초기 공통 후보: balanced-margin (단발 비교)

당시 공통 파일은 속도/가속도/저크 상한 20/6/10, 탐색 위치 제약 표현 2, 계획 거리 15m, 전진 거리 7.5m, 계획 반경 0.40m, 맵 팽창 4단계, 탐색 penna_attract=5e7이다. MPC 예측·갱신 간격은 각각 0.05초를 유지했다. 다른 패널티는 기존 사용자 설정의 값을 유지했다. 여러 항을 함께 조정한 후보이므로 개별 파라미터의 인과 효과를 확정하지 않는다.

Isaac GUI와 RViz2를 켠 새 세션에서 같은 파일 SHA256 `da1497d9d7bcc76071299cb03662cada208d6be66063c3268f7403842df129d9`로 두 장면의 임무 완료·착륙을 각각 한 번 확인했다.

| 장면 | 주행 sim s | 실제 max vel m/s | 실제 max acc m/s² | 실제 max jerk m/s³ | 장애물 여유 하한 m |
|---|---:|---:|---:|---:|---:|
| high_speed | 36.105 | 8.856 | 10.714 | 55.695 | 0.5363 |
| dense | 150.075 | 5.129 | 6.450 | 29.013 | 0.0040 |

실제 가속도와 저크는 약 0.21초 필터로 속도 벡터를 미분한 값이다. nominal 6/10을 실제가 초과했으므로 동역학 제한이 지켜진 설정이라고 표기하지 않는다. dense의 여유 하한 4mm는 양수지만 매우 좁아 작업상 비교 기준 0.1m를 충족하지 않는다. 실제 경유점 접근은 high_speed 0.113/0.357m, dense 0.811/0.198m로 임무 노드의 LIO 기준 도달 판정과 구분한다. dense 첫 목표는 0.8m 실제 접근 기준을 근소하게 넘겼다. 사용자 선호대로 LIO 오차 하나만으로 탈락시키지 않는다.

LIO 오차 max/RMS는 high_speed 0.730/0.322m, dense 1.187/0.471m. 기록된 odom stamp 최대 간격은 각각 5/30ms이고, 전체 기록에서 수신 최대 간격은 각 evaluation.json에 저장했다. dense 기록 평균 수신은 약 185Hz이며 이를 LIO 내부 발행 주파수와 동일하다고 단정하지 않는다.

이 후보는 하나의 파일로 두 장면을 주행한 비교 기준이다. 속도 상한 20m/s 도달, 충분한 dense 충돌 여유, 반복 신뢰성, 열린 직선 및 혼합 맵 검증은 확보하지 못했다. 최종 최적값으로 채택하지 않는다. 다음 조정에서는 dense의 실제 추종·통로 여유를 먼저 확보하고, 같은 설정을 유지한 채 두 장면을 반복 검증해야 한다. 100Hz 후보는 주기 숫자만 복사하지 않고 별도 이륙·호버 시험부터 제어기 응답을 확인한 뒤 재검토한다.

## 공통 후보 재검증 및 제어 어댑터 진단

후속 원본은 `log/super_flights/common-clearance-20261114/`에 보존했다. 위 balanced-margin의 단발 결과를 반복 안전성으로 해석하면 안 된다. 같은 계열의 후속 시험에서 목표 이동 후 대기, 좁은 장애물 여유, 실제 위치와 LIO 위치의 차이가 재현되었다. 현재 공통 파일 역시 검증 중인 출발점이며 최종 선정 파일이 아니다.

사용자 판정 조건은 원래 경유점 반경 **0.8m**, 정지 속도 **0.7m/s**를 그대로 유지한다. SUPER가 지도에 따라 안전 위치로 옮긴 목표를 원래 경유점 대신 인정하지 않는다. 임무 노드가 사용하는 LIO 기준 도달과 진단용 실제 기체 목표 접근을 함께 기록한다. LIO 최대 오차 0.5m는 별도 탈락 기준으로 사용하지 않는다. 장애물 여유 0.1m는 현재 비교 작업의 목표이며, 논문에서 가져온 보증값이 아니다.

### SUPER 변경 없이 확인한 연동 문제

SUPER는 FOLLOW_TRAJ/EMER_STOP에서 heartbeat를 발행하며, 정지 궤적 종료 후 WAIT_GOAL 등에서는 중단할 수 있다. 기존 어댑터는 이미 안전하게 멈췄어도 1초 뒤 착륙으로 전환했다. 선택 옵션 `allow_terminal_hold`는 궤적이 끝났고 종료 속도·가속도·저크가 모두 0이며, 실제 피드백이 종료점 0.3m 이내·속도 0.3m/s 이하인 경우에만 기존 정지 참조를 유지한다. 이동 중 heartbeat 상실, odom/PX4 stale 보호는 유지한다. 이 동작은 목표 도달 조건을 완화하지 않으며, 이동된 목표에 멈춰 원래 경유점 조건을 채우지 못한 시험은 여전히 미완주다.

기존 자체 착륙의 지면 높이 기준은 LIO 높이 오차로 모터 출력을 끝내 낮추지 못할 수 있었다. 선택 옵션 `use_px4_landing`은 PX4 NAV_LAND로 넘기고 rate 명령을 중단한다. 최신 PX4 착지 감지와 disarmed 상태가 모두 확인되어야 LANDED로 보고한다. GUI 시험에서 AUTO_LAND 진입과 착지·disarm을 확인했다. 두 옵션의 기본값은 false이며 기존 프로파일의 기본 동작은 유지한다. 공통 후보는 두 옵션을 명시적으로 활성화했다. 독립 DDS 시험 6개와 control_math 검사를 통과했으며 로그를 보존했다. SUPER 및 Super-LIO 알고리즘 C++은 이 과정에서 변경하지 않았다.

### 후보 파일을 분리해서 실행

```bash
./run_super_sim.sh high_speed
./run_super.sh --scenario high_speed --backend super_lio --profile common \
  --config /absolute/path/to/candidate.yaml --execute
```

`--config`는 common 프로파일에서만 허용하며, 같은 파일을 planner/controller/Super-LIO 설정에 전달한다. 기본 common 파일을 바꾸지 않고 후보를 비교하기 위한 실행 옵션이다. 실제 로딩한 파일은 각 실행의 `loaded-common.yaml`로 보존한다. dense는 scenario만 바꾸며 후보 파일은 동일하게 유지한다.

### 물리 여유와 실제 주기

`exact_clearance.py`는 실제 장면 생성에 사용한 PCD SHA256과 점유 voxel 수를 확인한 뒤, 기체 중심 기록 경로에서 실제 5cm voxel 합집합까지의 거리를 계산한다. 반경 0.33m 기체 구와 1cm 이하 경로 보간 표본의 0.005m 여유를 뺀다. KD 탐색은 미조회 voxel이 최소 거리를 줄일 수 없는지 검증하며, 무작위 표본의 전수 거리 계산과도 비교했다. 이는 기록된 위치의 보간 경로에 대한 여유이며 연속 PhysX 접촉 센서 검출을 뜻하지 않는다. 기존 PCD 재구성 오차를 한 번 더 빼는 보수적 지표도 비교용으로 남긴다.

nominal 제어 주기는 실제 달성 주파수와 다르다. 0.01초 갱신/예측, R_pitchroll=32 후보는 호버 RMS 0.033m였지만 실제 명령 약 66Hz/sim s였고, 두 장면 주행에서 추종 실패했다. 갱신 0.01초·예측 0.05초 후보는 호버에서 불안정했다. 0.02초 갱신/예측, R_pitchroll=16 후보는 호버 RMS 0.019m, 최대 0.034m, 실제 명령 약 34Hz/sim s였으며 주행 평가를 별도로 수행한다. 호버 성공을 임무 완주로 집계하지 않는다.

현재 H=15m, a=6m/s², j=10m/s³에서 정지점까지의 jerk 제한 제동 거리는 20m/s에 약 39m이며 감지·계획·추종 지연 여유가 추가로 필요하다. 따라서 max_vel=20을 설정한 것만으로 해당 속도 도달을 기대할 수 없다. 제어 안정성과 두 장면의 여유를 확보한 뒤 계획 거리와 충분한 개방 구간을 함께 검증해야 한다. 이 계산만으로 필요한 가시 거리 전체나 안전 비행을 보장하지 않는다. 혼합 맵과 최종 반복 검증은 아직 완료하지 않았다.

### 제어 0.02초 후보와 저크 패널티 비교

원래 경유점 0.8m 조건을 유지한 새 GUI 세션에서 다음 세 공통 후보는 각각 high_speed/dense 모두 임무 노드의 왕복 완료와 착륙·disarm을 기록했다. 다만 단발 결과이며 안전 최적값으로 선정하지 않았다. `control50-r16`은 목표 주기 명칭으로 실제 50Hz 달성을 뜻하지 않는다.

| 후보 | high_speed 실제 max vel / acc / jerk | dense 실제 max vel / acc / jerk |
|---|---:|---:|
| control/prediction 0.02초, R_pitchroll=16, 반경 0.40m | 8.50 / 7.16 / 41.47 | 7.82 / 7.43 / 98.11 |
| 위 후보의 계획 반경만 0.45m | 9.03 / 6.31 / 31.18 | 6.55 / 7.02 / 52.64 |
| 반경 0.45m 후보의 exp/backup penna_jerk를 각각 1e10→1e11 | 8.65 / 6.55 / 34.98 | 7.47 / 6.65 / 31.25 |

단위는 m/s, m/s², m/s³이며 실제 미분 값은 약 0.21초 필터를 사용한다. 마지막 후보의 실제 경유점 접근 거리는 high_speed 0.563/0.530m, dense 0.726/0.427m였다. LIO max/RMS는 각각 0.960/0.403m, 1.305/0.639m다. 표본화한 계획 최대 jerk는 high_speed 10.10, dense 25.76m/s³였고, 이전 반경 0.45m 후보의 dense 계획 jerk는 39.01m/s³였다. 표본화 최대값은 연속 궤적의 해석적 최대값이 아니며, 단발 차이만으로 인과 효과나 반복 개선율을 확정하지 않는다.

### 기체 외접 반경 지표의 정밀화

이전에 사용한 R0.33m 구는 여유를 더한 보수적 비교 구였다. 현재 USD SHA256 `73781389d524fddf56d9a3421fda0bcb705f6dabfe252e40faa2d7f3747b17a4`를 현재 `configure_zav_collision`로 메모리의 session layer에만 구성해 다시 측정했다. 활성 collider 37개, 다른 활성 collider 0개를 확인했다. body 원점 기준 collider 정점 최대 반경은 0.249322m이며 authored contact offset 0.002m를 포함하면 0.251322m다. 이를 R0.26m로 올림한 CAD 외접 구 지표를 추가했다. 원본 USD와 물리 형상은 바꾸지 않았다. 결과는 `current-collision-geometry.json`, 재현 코드는 `audit_collision_geometry.py`에 있다.

R0.33m 지표는 지우지 않았으며, 음수를 실제 충돌로 판정하지 않는다. R0.26m의 추가 지표에서도 경로 표본 여유 0.005m를 뺀다. 마지막 저크 후보의 여유는 high_speed 0.429m, dense 0.025m였다. 이는 authored convex hull/decomposition 형상의 외접 구와 기록 경로를 사용한 기하 지표이며, 실제 PhysX cooked geometry 및 접촉 보고를 직접 검증한 것은 아니다. 현재 작업상 0.1m 여유 목표는 CAD 지표로 비교하되 이전 구 기준 수치도 병기한다. dense 2.5cm는 충분한 여유와 반복성을 확보했다고 보기 어렵다.

최신 비교 진입점은 `log/super_flights/common-clearance-20261114/control-candidate-comparison.md` 및 같은 이름의 JSON이다. 기본 공통 YAML은 기존 20Hz 실험 기준에 두고, 고주기 후보를 검증 없이 기본값으로 바꾸지 않았다. 기존 시나리오 YAML과 LIO 기본 설정 파일의 SHA256 보존도 재확인했다.

### penna_attract 추가 비교와 이번 회차 판단

저크 패널티 1e11 후보에서 exp penna_attract만 5e7→5e8로 높였다. high_speed는 왕복·착륙했고 실제 peak 7.79m/s, 주행 40.71s, 실제 목표 접근 0.505/0.619m, CAD 여유 0.563m였다. dense는 중간 backup 정지 뒤 다시 진행했지만, 최종적으로 첫 목표가 원래 목표에서 1.260m 떨어진 곳으로 옮겨진 채 WAIT_GOAL에 머물렀다. 원래 0.8m 조건을 유지했으며 미완주로 기록하고 착륙을 요청했다. `operator-stop.json`에 당시 LIO 위치·속도와 궤적 종료점을 보존했다. 실패까지의 peak는 완주 성능으로 비교하지 않는다. dense 최종 CAD 여유는 0.049m로, 초반의 0.111m보다 좁아졌다.

이번 회차에서 실제 경유점 접근까지 두 장면 모두 0.8m 안에 들어온 공통 후보는 `candidates/control50-r16-r045-jerk10.yaml`이었다. 그러나 dense 여유 0.025m, 실제 jerk 제한 초과, 반복 시험 부재 때문에 최종 공통 최적값으로 선정하지 않았다. nominal 20m/s도 달성하지 못했다. 다음 검증에서는 이 후보의 반복성과 여유 확보를 먼저 확인해야 하며, 그 전에는 혼합 맵이나 20m/s 가속 시험을 통과한 설정으로 배포하지 않는다. 원래 경유점 조건·SUPER/Super-LIO 알고리즘·장면별 전환 없는 실행은 유지했다.

## 후속 검증 — 2026-11-15

원본·후보 YAML·GUI 캡처·실행 결과·kinematics·수신 이벤트는 `log/super_flights/common-penalty-20261115/`에 보존한다. `comparison.md`와 `comparison.json`이 비교 진입점이다. 기존 임무의 원래 경유점 반경 0.8m와 정지 속도 0.7m/s는 유지했다. Ground truth는 오프라인 진단에만 사용하며, LIO 오차 0.5m를 임의의 독립 탈락 기준으로 적용하지 않는다.

### 반복 실행과 추가 비교

이전 저크 후보(exp/backup penna_jerk=1e11, penna_attract=5e7, 계획 반경 0.45m, H15, control/prediction 0.02초, R_pitchroll=16)를 **파일 SHA까지 동일하게** 재시험했다. 이전 회차와 합쳐 두 장면 모두 각각 2회씩 LIO 임무 왕복 완료와 착륙을 기록했다. high_speed는 42.71/41.31s, 실제 peak 8.65/8.28m/s였다. dense는 118.63/164.86s, peak 7.47/7.62m/s였다. 반복 완주가 충분한 여유나 최적값을 뜻하지는 않는다.

실제 경유점 최근접 거리도 저속 조건과 분리해 보강했다. 이전 dense 첫 목표의 전체 통과 최근접은 0.726m였지만, 실제 속도 <0.7m/s 조건에서는 0.970m였다. 새 high_speed 재시험의 실제 저속 복귀 목표 접근은 3.095m였고 LIO max/RMS는 2.843/1.307m였다. 이는 기존 LIO 도달 판정을 바꾼 것이 아니라 추가 진단이다. 실제 목표 저속 거리, 수신 간격, 명령 갱신율, 경로 길이와 평균 이동 속도를 evaluation.json에 추가했다.

주요 추가 비교는 다음과 같다. 단발 차이로 인과 개선율을 확정하지 않는다.

- exp penna_attract만 5e7→1.5e8: dense 왕복 151.65s, peak 4.63m/s, 구 R0.26 여유 0.058m. high_speed는 복귀 중 실제 위치가 시험 영역을 벗어나 예방 중단했다. 이 실행은 실제 충돌 검출 또는 필연적 임무 실패를 입증한 것이 아니라, 완주 성능이 확보되지 않은 실행이다.
- 이전 penna_attract=5e8/jerk=1e11 후보에서 계획 반경만 0.45→0.40m: high_speed 40.86s, peak 8.85m/s, 구 여유 0.851m. dense는 256.64s, 실제 경로 약 502m, peak 6.97m/s, 구 여유 0.044m였다. 후반의 긴 구간은 backup hold가 아니라 FOLLOW_TRAJ에서 우회와 재계획을 반복한 구간이었다. 위치 오차는 작았지만 빠른 공통 프로파일로 유리하지 않았다.
- 반복 저크 후보에서 exp penna_t만 3e7→1e8: dense 122.24s, peak 5.81m/s, 구 여유 0.045m. high_speed는 실제 시험 영역 이탈로 예방 중단했다. 큰 시간 패널티가 두 장면의 개선을 입증하지는 못했다.

### 계획 거리와 A* 창의 의존성

H15→45m만 바꾼 첫 실행은 두 장면 모두 첫 목표 계획 중 `seed line too long` 예외로 종료되었다. 기존 A* map_voxel_num=[500,500,100]에서 해상도 0.15m의 XY 반폭은 37.5m다. 코드에서 로컬 목표가 창 경계로 투영된 뒤 REACH_GOAL로 반환되면 PathSearch가 원래의 먼 목표를 덧붙일 수 있어, corridor seed가 허용 길이를 넘는 설정 조합이 된다. 실패를 속도 성능으로 집계하지 않았다.

A* 창을 [800,800,100](XY 반폭 60m)으로 늘린 H45 후보는 출발했지만 high_speed는 backup 종료 후 endpoint 오차 0.484m, LIO 속도 0.288m/s, 궤적 종료 1.031초 후 heartbeat 보호 동작으로 미완주했다. 정지점 유지 보호의 endpoint 0.3m 조건은 완화하지 않았다. 계획 표본 peak도 8.99m/s로 기존보다 크게 증가하지 않았다. dense는 복귀 계획에서 A*의 0.1초 시간 제한과 재계획 시간 초과(로그상 약 0.25~0.5s)를 반복해 360초 제한으로 종료되었다. 알고리즘 코드는 수정하지 않았다. 큰 계획 거리에는 탐색 창뿐 아니라 계산 예산 의존성도 있다.

그 뒤에는 반복 저크 후보에서 H15→25m만 바꾼 중간값 비교를 수행한다. 기존 A* 반폭 37.5m를 그대로 유지할 수 있으며, 다른 패널티와 controller 값은 동일하다. 결과가 비교 JSON에 추가되기 전에는 이 후보를 검증 완료로 취급하지 않는다.

### 실제 수신 주기와 CAD 형상 진단

200Hz stamp가 균등한 wall-clock 수신을 뜻하지 않았다. 새 high_speed 기록에서는 LIO stamp 간격이 거의 5ms였지만 수신 간격 중앙값은 약 1.58ms, p95 약 38ms였다. dense도 p95 약 38ms의 묶음 수신이 있었고 최대 간격은 약 65ms였다. 명령 관측은 대체로 sim 시간당 32~34회였다. 현재 GUI 루프는 physics 200Hz/render 20Hz이므로, nominal 주기만 보고 실기 MPC 100Hz와 같다고 할 수 없다. 시뮬레이터 timing과 렌더 설정은 이번 비교에서 변경하지 않았다. `receive-timing.json`과 `attitude-drift-diagnosis.json`을 보존했다.

구 지표의 과보수성 여부도 별도로 확인했다. 원본 USD를 저장하지 않고 활성 collider 37개의 정점 42,748개를 body frame으로 변환해, 전체를 포함하는 외접 convex hull(319 vertices/634 faces)을 만들었다. contact offset을 포함할 수 있도록 기존 R0.26 구까지의 차이 0.010678m를 padding으로 유지했다. 기록 자세를 Slerp로 보간하고 평행이동 0.005m 여유와 회전 보간 여유를 추가로 차감한다. voxel과 hull의 분리 축 투영은 Cauchy–Schwarz에 의해 거리의 하한이며, body face/voxel face/center ray 방향 중 가장 강한 하한을 사용한다. 독립적인 해석 AABB 1,000개와 회전 box convex QP 200개 비교에서 하한이 실제 거리보다 커지는 오차는 최대 1.12e-16m였다.

이 진단에서도 이전 dense의 여유 하한은 0.026m, 새 동일 후보 재시험은 0.053m로 좁았다. 좁은 여유를 단순한 외접 구 계산 문제로 설명하지 않는다. 기존 R0.33/R0.26 값도 삭제하지 않았다. 이 결과는 authored hull 및 기록 경로/자세 보간에 대한 기하 하한이며, runtime PhysX cooked geometry와 실제 contact 보고를 직접 검증한 것은 아니다. 자세한 수학 검증과 원본 SHA는 `projection-verification.json`, `body-hull-audit.json`, 각 실행의 `hull-clearance.json`에 있다.

### 재현용 공통 비교 기준 파일

`src/zav_super/config/common_super_lio_reference.yaml`을 추가했다. 두 장면에서 각각 2회씩 읽어 사용한 후보와 바이트까지 동일한 파일이다(SHA256 `1140f92c21d4b1454b77acca5160a64b98fdf755bed6e7afc3cb96d0abbe7148`). 최종 안전 최적값으로 선정한 파일은 아니며, 기존 common/high_speed/dense YAML은 보존한다.

```bash
./run_super_sim.sh high_speed
./run_super.sh --scenario high_speed --backend super_lio --profile common \
  --config "$PWD/src/zav_super/config/common_super_lio_reference.yaml" --execute
```

양쪽 명령의 scenario만 dense로 바꾸면 같은 YAML로 밀집 환경을 시험한다. 이 단계 당시 `--config`를 생략한 common은 20Hz 실험 기준 파일을 사용했다. 현재 기본값은 후속 6회 접촉/추종 검증 기준으로 갱신했다. reference의 nominal 50Hz는 실제 50Hz 명령 달성을 뜻하지 않으며, GUI 관측은 약 32~34회/sim s였다.

### 25m 후보 결과와 이번 회차 상태

H25 후보는 두 장면 모두 기존 LIO 조건으로 왕복·착륙했다. high_speed 주행 34.885s, 실제 max v/a/j=9.552/7.534/46.687; dense 주행 169.450s, 실제 max v/a/j=6.760/6.473/65.404였다. high_speed는 H15 반복 후보의 41~43s보다 짧았지만 dense 성능과 실제 jerk가 함께 좋아진 것은 아니다. dense의 R0.26 구 여유는 0.0131m, 자세를 보강한 하한은 0.0182m로 좁았다. 안전 최적값으로 채택하지 않았다. nominal max_vel=20m/s 도달도 확인하지 못했다.

왕복·착륙을 완료한 실행들은 TRACKING뿐 아니라 착륙까지 포함한 전체 기록의 장애물 여유도 보강 계산했다. 바닥 평면의 의도된 착지 접촉은 장애물 voxel 거리에서 제외하며, 실제 body 최종 높이는 모두 약 0.095m로 확인했다. 전체 계산에서도 기존 반복 후보 dense 하한은 0.0263/0.0526m, H25 dense는 0.0182m였다. `hull-full-flight-clearance.json`과 `full-flight-geometry-summary.json`에 원본 SHA, 자세 보간 여유, 최종 실제 위치를 보존했다. runtime contact 보고를 직접 측정했다는 뜻은 아니다.

이번 회차는 새 GUI 실행 14회(그중 H45의 첫 두 실행은 TRACKING 전에 예외 종료)를 포함한다. 변경한 설정의 출발 실패·보호 동작·시간 제한도 비교에서 삭제하지 않았다. 기존 common 기본값은 검증 없이 교체하지 않았고, 반복 완주 파일은 별도 reference로 보존했다. 빠른 공통 최적값 탐색은 미완료다. 다음 단계에서는 실제 명령 갱신과 reference 추종 오차가 만드는 dense 여유 손실을 분리해 개선해야 하며, 같은 하나의 YAML·같은 GUI 조건으로 두 장면을 다시 평가해야 한다. 원래 경유점 조건을 완화하거나 환경별 전환 로직을 추가하는 방식은 사용하지 않는다.

### 접촉 직접 계측 및 후속 반복 시험

후속 GUI 측정에서 reference YAML의 dense 완주 중 native PhysX 접촉 응답 두 구간을 확인했다. 임무 완료와 보간 기하 하한만으로 무접촉을 보증할 수 없으므로 reference는 계속 재현용 비교 설정으로만 사용한다. `docs/super-contact-tracking.md`와 `log/super_flights/contact-tracking-20261115/comparison.md`에 직접 접촉·추종/LIO 오차·실패 실행을 함께 보존한다.

Q_pos_xy만 300→600으로 높인 공통 후보는 high_speed 3/3, dense 2/3 완주였다. dense 실패는 LIO가 계속 수신되는 동안 약 0.90초 재계획과 1.10초 궤적/heartbeat 수신 공백을 동반한 보호 착륙이었다. 이후 exp_traj.opt_accuracy만 5e-6→1e-4로 바꾼 공통 후보를 별도 비교한다. 이는 상대 비용 수렴 기준이며 원래 경유점 0.8m 조건은 바꾸지 않는다. 반복 검증 전에는 기존 common 기본 파일을 교체하지 않는다. SUPER/Super-LIO C++나 환경별 전환 로직도 변경하지 않았다.

### 후속 공통 기준과 고속화 옵션 결과

현재 common 기본값은 `radius050-time5e7` 새 GUI 6회 검증 기준이다. exp penna_t=5e7, exp opt_accuracy=1e-4, robot_r=0.50, Q_pos_xy=600, planning_horizon=15m이며 이전 common은 `common_super_lio_20hz_reference.yaml`로 보존했다. install 파일도 동일 SHA로 갱신했다. 실제 peak는 high_speed 7.323~7.655m/s, dense 5.597~7.214m/s였고 최소 CAD 여유 하한은 0.139m였다. 가장 빠른 공통 최적값 또는 실제 20m/s 도달을 선언한 것은 아니다.

기존 goal_vel_en만 true로 켠 비교는 high_speed heartbeat 보호 착륙 및 dense 출발 실패로 기각했다. planning_horizon만25m로 늘린 비교는 high_speed peak10.779m/s/주행44.110s를 관측했지만 dense가 원래 목표의0.8m 조건 밖에서 대기해 미완주로 기각했다. 목표를 옮기거나 원래 조건을 완화하지 않았다. 상세 기록과28회 목록은 `docs/super-contact-tracking.md` 및 `log/super_flights/contact-tracking-20261115/`에 보존한다. 환경별 전환 로직과 SUPER/Super-LIO C++는 변경하지 않았다.

### 혼합 맵 후속 평가

`mixed` 맵과 common 프로파일 실행을 추가했다. 같은 common YAML을 그대로 사용한 새 GUI 3회는2회 무접촉 왕복·착륙,1회 밀집 구간 접촉 응답 후 탐색 정체로 미완주였다. 따라서 기존 두 장면에서의6회 통과를 혼합 환경 안정화의 증거로 확대하지 않는다. 맵과 실행 명령, 성공·실패 전체 결과는 [혼합 맵 안내](super-mixed-map.md)에 있다. 이 평가에서 common/legacy YAML이나 SUPER/Super-LIO 알고리즘을 수정하지 않았다.
