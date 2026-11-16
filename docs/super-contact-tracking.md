# SUPER 접촉 계측과 추종 오차 분리

최신 자료는 `log/super_flights/contact-tracking-20261115/`에 보존한다. `comparison.md`와 `comparison.json`에서 주행·접촉·오차·계측 부하를 함께 비교한다. 기존 경유점 0.8m, 정지 0.7m/s, Super-LIO만 사용하는 조건과 Isaac Sim GUI + RViz2 조건을 유지했다. SUPER와 Super-LIO 알고리즘 코드는 변경하지 않았다.

## 현재 채택 범위

현재 `common_super_lio.yaml`은 `radius050-time5e7`과 동일한 공통 실험 기준으로 갱신했다. high_speed/dense 각각 새 GUI 세션 3회 모두 정상 완주·착륙, 외부 접촉 보고 0건, 전체 CAD 여유 하한 최소 0.139m였다. 이전 common은 `common_super_lio_20hz_reference.yaml`로 보존했다. 최고속도 최적값으로 선정한 것은 아니며 실제 20m/s, nominal acc/jerk의 강제 준수, 혼합 맵은 검증하지 못했다. 아래 기록의 “기본값을 아직 교체하지 않았다”는 각 후보의 당시 상태다.

## 실제 접촉 관측

이전에는 기체 CAD와 기록 경로의 거리 및 임무 완료를 확인했지만, PhysX contact 보고를 직접 수집하지 않았다. 이번 계측에서 기존 reference YAML로 dense 왕복·착륙을 완료한 실행에서도 두 구간의 접촉 응답을 확인했다.

- 약 25.555~25.595 sim s: 최대 contact impulse norm 약 0.120. 보고된 separation은 양수인 응답도 포함한다.
- 약 143.215~143.220 sim s: 최대 impulse norm 약 0.343, 최소 separation 약 −0.00109m. 음의 separation과 실제 접촉 응답이 함께 있었다.

`on/dense/contacts.jsonl`에는 근접/유지/해제 보고가 총 403개 있지만 이것을 403회 충돌로 세지 않는다. 그중 impulse가 0이 아닌 보고는 4개이며, 0.25초 간격으로 묶으면 2개 응답 구간이다. 양의 separation과 0 impulse인 근접 보고는 분리한다. 필터링한 acc/jerk 그래프만으로 무접촉을 판정하지 않는다.

## 선택적 계측 실행

```bash
# 매 실행마다 새 출력 경로를 사용한다. 기존 파일에 덧붙이지 않는다.
ZAV_CONTACT_REPORT_PATH="$PWD/log/my_super_trial/contacts.jsonl" \
  ./run_super_sim.sh high_speed

# 다른 터미널. 실험할 공통 YAML을 명시한다.
./run_super.sh --scenario high_speed --backend super_lio --profile common \
  --config "$PWD/src/zav_super/config/common_super_lio_reference.yaml" --execute
```

두 명령의 scenario만 dense로 바꾸면 같은 설정을 사용한다. reference는 재현용 비교 기준이며 무접촉 최적값으로 선정한 파일이 아니다. 계측 환경변수가 없으면 기존 기본 실행에서는 ContactReportAPI를 추가하지 않는다. contact API는 runtime stage의 기체 rigid body에만 적용하며 질량, 관성, 충돌 형상, 물리 200Hz/렌더 20Hz와 controller 출력 경로는 바꾸지 않는다. 파일을 원본 USD로 저장하지 않는다.

기록기는 바닥과 자기 접촉은 집계만 하고 외부 접촉은 collider/actor 경로, 시각, separation, impulse를 기록한다. 물리 callback에서 디스크를 쓰지 않고 메인 루프에서 버퍼를 비운다. 큰 보고에서 첫 8개 contact point 밖에 최대 impulse가 있을 수 있어, 후속 기록에는 최대 impulse 점과 최소 separation 점도 별도로 보존한다. 초기 on 기록의 첫 점 법선을 최대 충격 방향으로 해석하지 않는다.

`callback_errors`, `dropped_events`, 바닥 접촉 양성 확인, 이륙 후 floor_lost와 착륙 후 floor_found, 마지막 요약의 시각 커버리지를 확인한다. 무효 계측이나 계측을 끈 실행은 장애물 0건으로 판정하지 않는다. 바닥 접촉은 착륙에 필요한 접촉이며 장애물 접촉과 구분한다.

## 계측 검증과 관측 부하

`simulator/tests/check_contact_diagnostics_gui.py`는 별도 GUI 장면에서 두 물체를 바닥과 장애물에 떨어뜨린다. 두 종류의 접촉을 실제 검출했고 callback 오류/유실 0으로 통과했다. 비행 성능 시험으로 집계하지 않는다. 일반 Python의 분류/유효성 회귀 검사는 다음과 같다.

```bash
python3 simulator/tests/test_contact_diagnostics.py
```

네 번째 검사는 최대 impulse가 첫 8개 이후에 있어도 보존되는지 확인한다. 실제 baseline contact 보고에서 발생한 데이터 잘림 문제를 대상으로 한 회귀 검사다.

같은 YAML의 fresh GUI off/on 한쌍 비교에서 LIO 수신 p95는 high_speed 38.15→38.27ms, dense 38.22→38.45ms였다. 명령 관측은 각각 sim 초당 34.61→34.07회, 32.86→32.78회였다. 이것은 성능 영향이 없다는 통계적 보증이 아니다. callback 시간에는 native PhysX 보고 생성 비용이 포함되지 않고, 주행 시간에는 경로·접촉·LIO 실행 편차도 영향을 준다. 모든 off/on 실행에서 동일한 궤적·지도 스냅샷 observer를 사용했다.

## 계획·추종·LIO 오차의 분리

observer는 LIO/ground truth의 시각·수신 시각, 발행된 모든 position trajectory, 지도 입력과 ROG occupied/inflated 시각화 스냅샷을 저장한다. 실제 장면이나 ground truth를 planner/controller에 주입하지 않는다.

같은 시각의 벡터 항등식은 다음과 같다.

```
reference - truth = (reference - LIO) + (LIO - truth)
```

벡터의 크기들은 더해지지 않는다. observer가 마지막으로 받은 궤적을 평가하며 controller 내부의 직접 계측은 아니다. `reference-switch-diagnosis.json`에서 off 기준 대부분의 궤적 전환은 같은 시각 위치 점프가 수치 오차 수준이었고, 미래 start 시각으로 점프하는 일반적인 문제는 관측되지 않았다.

같은 시각의 거리뿐 아니라 가까운 과거 reference 경로까지의 최소 공간 거리도 계산했다. 접촉 응답 시점에는 같은 시각 추종 오차 약 0.35~0.50m, reference 이력까지의 거리 약 0.31~0.45m가 관측되었다. 단순한 시간 지연만으로 설명할 수 없는 공간 차이가 있었지만, 이것만으로 독립적인 인과 기여도를 확정하지는 않는다. 지도 스냅샷의 나이도 보고하며, 시각화 창이 정확한 전체 planner collision query를 대신한다고 해석하지 않는다.

`error-separation.png`, `decomposition.npz`, `near-obstacle-diagnosis.json`과 원본 contact 파일을 함께 본다. 새 candidate는 planner/LIO/속도 상한을 유지하고 MPC 수평 위치 비용 `Q_pos_xy`만 300→600으로 바꿨다. GUI 호버에서 실제 위치 RMS 0.0127m, 최대 0.0291m, 정상 착륙을 확인했다. 첫 두 장면 주행에서는 장애물 보고 0건이었고 dense 추종 p95 약 0.44m, 전체 CAD 자세 여유 하한 약 0.11m였다. 실제 dense max acc/jerk는 9.37m/s²/57.65m/s³로 nominal 6/10 상한을 지킨 설정은 아니다. 반복 시험이 끝나기 전에는 최종 공통값으로 선정하지 않는다.

## 반복 시험에서 드러난 계산 지연

`position600`을 같은 SHA의 파일로 새 GUI 세션에서 반복했다. high_speed는 3회 모두 정상 완주·착륙했고, dense는 3회 중 2회만 완주했다. 모든 실행에서 장애물 보고는 0건이었지만, `position600-repeat2/dense`는 경로 중간에 `SUPER trajectory heartbeat expired`로 보호 착륙했다. 따라서 Q_pos_xy=600만 바꾼 후보를 검증 완료 설정으로 선정하지 않는다.

실패 시점에 기존 궤적은 약 1.58초 남아 있었고, 기체는 endpoint에서 약 3.07m 떨어져 약 3.60m/s로 이동 중이었다. 정상 정지점 유지 조건에 해당하지 않는다. 궤적·heartbeat 수신에는 약 1.098초 공백이 있었지만 그동안 LIO 248개가 계속 수신되었고 최대 LIO 수신 공백은 약 47ms였다. 재계획 로그에는 약 0.90초 계산 초과가 있었다. 이 실행을 odometry 단절이나 완료 후 대기로 분류하지 않았다.

현재 SUPER ROS2 앱은 SingleThreadedExecutor를 사용한다. replan과 command/heartbeat timer의 callback group이 분리되어 있어도 실행 스레드는 하나이므로, 긴 재계획이 heartbeat 발행을 지연시킬 수 있다. LBFGS `opt_accuracy`는 상대 비용 수렴 조건이며 명시적인 wall-clock 계산 제한이 아니다. `replan_forward_dt`를 넘었다는 확인도 계산을 끝낸 뒤 이뤄진다. 이 코드 경로와 관측은 계산 지연 해석을 뒷받침하지만, subscriber 기록만으로 각 callback의 정확한 실행 시간을 직접 측정한 것은 아니다.

후속 `accuracy1e4` 후보는 `position600`에서 **exp_traj.opt_accuracy만 5e-6→1e-4**로 바꿨다. 원래 경유점 위치 허용 오차, heartbeat timeout, terminal hold 조건, backup opt_accuracy, 속도 상한, 패널티, LIO 설정은 그대로다. 상대 비용 수렴 기준이 느슨해져 계산을 줄일 가능성을 시험하는 것이며, 유효한 해나 계산 시간 상한을 보장하는 변경은 아니다. 알고리즘 C++와 executor 코드는 수정하지 않았다.

첫 새 GUI 쌍은 두 장면 모두 원래 조건으로 정상 완주·착륙했다. 실제 최고속도는 high_speed 7.812m/s, dense 7.044m/s였고 native 장애물/미분류 외부 접촉 보고는 모두 0건이었다. 기록 전체 CAD 여유 하한은 각각 0.666m, 0.150m 이상이었다. dense의 최대 궤적·heartbeat 수신 공백은 약 0.462초였으며, LIO 최대 위치 오차는 high_speed 0.476m, dense 0.804m였다. 이 한 쌍만으로 안정성을 확정하지 않고 같은 YAML로 새 세션 반복을 진행한다.

`runtime-audit.json`은 TRACKING 경계와 겹치는 메시지 공백 및 바닥 found/lost 집계 변화의 시각 구간을 보존한다. 실패 직전부터 fault 직후까지 걸친 공백도 포함한다. 바닥 전환 구간이 주행 시간과 겹치면 보수적으로 추가 확인 대상으로 표시한다. 집계 전환이 없다는 사실을 짧은 모든 접지의 부재 증명으로 해석하지 않는다.

## max_acc/max_jerk를 실제 측정값과 함께 보는 이유

현재 소스의 exp/backup 최적화는 샘플 지점에서 `||jerk||² - max_jerk²` 등의 초과를 smoothed-L1 비용에 넣는다. 비용을 크게 준다고 연속 시간의 모든 실제 기체 운동을 강제 제한하는 구조는 아니다. 특히 `exp_traj_optimizer_s4.cpp`의 최종 수락 조건에는 jerk 초과 검사가 없고 velocity 검사는 주석 처리되어 있다. backup도 최종 penalty 검사에 jerk 항이 없다. 따라서 max_jerk=10을 설정한 것만으로 계획 jerk의 모든 초과까지 거부한다고 해석하지 않는다. 기체의 실제 미분값에는 궤적 전환, 추종 오차, 물리 응답이 추가된다.

실제 v/a/j 최대값과 별도로 reference 표본 최대값도 `reference-assessment.json`에 보존한다. 이 reference 값은 약 10Hz 평가이므로 해석적 극값이 아니며 실제 acc/jerk는 약 0.21초 필터 미분값이다. 서로 다른 계산의 최대값을 같은 순간의 값처럼 빼거나, 필터 jerk만으로 접촉 유무를 판정하지 않는다. 원본 궤적과 고빈도 odometry를 남겨 후속 정밀 분석이 가능하도록 했다. 이 조사에서는 해당 수락 조건이나 알고리즘을 수정하지 않았다.

### accuracy1e4 3회 반복 결과

동일 SHA256 `66233df2343c8b7a2270714c6cc4c5b679a82c1defb2116f2fb15dd5a3a679b3`로 high_speed와 dense를 각각 새 세션에서 3회씩 시험했다. 원래 경유점 조건으로 6회 모두 정상 완주·착륙했고, 모든 실행에서 장애물/미분류 외부 접촉 보고 0건, callback 오류/유실 0이었다. 주행과 겹치는 바닥 found/lost 전환 집계 구간도 없었다. 최대 궤적·heartbeat 수신 공백은 전체 6회 중 약 0.538초였다.

| 장면 | 주행 시간 sim s (3회) | 실제 최고속도 m/s (3회) | 전체 CAD 여유 하한 m (3회) | LIO 최대 오차 m (3회) |
|---|---|---|---|---|
| high_speed | 43.295 / 45.900 / 43.105 | 7.812 / 7.771 / 8.285 | 0.666 / 0.413 / 0.710 | 0.476 / 0.541 / 3.668 |
| dense | 116.440 / 138.635 / 120.645 | 7.044 / 4.472 / 6.137 | 0.150 / 0.144 / **0.0665** | 0.804 / 0.713 / 1.319 |

3회 통과가 모든 환경의 안전이나 heartbeat 상한을 보장하지는 않는다. 마지막 dense는 직접 접촉 보고는 없었지만 이번에 사용한 **기하 여유 하한 0.1m 작업 목표**에는 못 미쳤다. 이는 사용자가 지정한 위치 정확도 cutoff가 아니며, LIO 오차가 0.5m를 넘었다는 이유로 탈락시킨 것도 아니다. `accuracy1e4-qualification.json`에 통과/미통과 검사를 전부 보존했다. 기존 common 기본값은 교체하지 않았다.

계획했던 `time5e7`(exp penna_t 3e7→5e7) 파일은 준비만 하고 실행하지 않았다. 그 활성화 조건인 6회 전체 여유 검사를 통과하지 못했기 때문이다. 다음 `radius050` 비교는 accuracy1e4에서 **super_planner.robot_r만 0.45→0.50m**로 바꾼다. 실제 차량 CAD를 키우는 것이 아니라 planner의 기체 반경을 늘리는 것이다. 패널티, horizon, controller/LIO, opt_accuracy, 원래 경유점과 heartbeat 보호 조건은 그대로다. 여유가 부족한 상태에서 시간 비용을 높이는 실험보다 이 조건을 먼저 검증한다.

### radius050 첫 쌍과 다음 시간 비용 비교

radius050 첫 쌍은 두 장면 모두 정상 완주·착륙했고 장애물/미분류 외부 접촉 보고가 0건이었다. high_speed는 주행 43.530s, 실제 max v/a/j=7.870/7.156/38.064, CAD 여유 하한 1.136m였다. dense는 주행 **223.835s**, max v/a/j=4.182/5.928/35.853, CAD 여유 하한 **0.11875m**였다. LIO 최대 오차는 각각 1.219m/1.179m였다. 한 실행에서 여유 목표를 통과했지만 dense 주행이 느려, 이 설정을 그대로 빠른 공통 최적값으로 채택하지 않는다. 반경 변화의 인과 효과를 한 쌍으로 확정하지도 않는다.

이후 실제 실행 후보는 `radius050-time5e7`이다. **radius050에서 exp_traj.penna_t만 3e7→5e7**로 바꾸며, 준비만 한 원래 `time5e7`(반경 0.45 기반)과 구분한다. 반경 0.50 설정의 두 장면 첫 여유 검사를 통과한 뒤 시간 비용을 비교하기로 계획을 갱신했다. radius050 자체는 3회 반복으로 검증된 baseline이 아니므로 새 시간 비용 후보도 별도 반복 없이 최종 채택하지 않는다. 모든 실패와 좁은 여유는 비교에 남기고, 경유점/heartbeat/terminal hold 보호 조건은 완화하지 않는다.

`accuracy1e4-motion-repeats.png`는 반경 0.45 후보의 같은 YAML 3회씩에 대해 실제 v/a/j, 각 최대값 및 nominal bound를 한 그림에 표시한다. 각 최대값이 실행별 evaluation.json과 일치하는지도 검사했다. 전체 원본 CSV 및 실행별 HTML 보고서는 개별 telemetry 디렉터리에 있다.

### radius050-time5e7 첫 쌍

동일한 반경 0.50m에서 exp penna_t만 5e7로 높인 첫 쌍은 두 장면 모두 정상 완주·착륙, 외부 접촉 보고 0건이었다. high_speed는 44.725s, 실제 max v/a/j=7.655/6.244/29.464, CAD 하한 0.808m였다. dense는 **130.435s**, max v/a/j=**7.214/6.360/23.870**, CAD 하한 **0.13925m**였다. 이 한 실행에서 dense 시간이 줄었지만, high_speed 최고속도는 커지지 않았으며, 실행 편차와 LIO·경로 변화가 있어 시간 비용의 독립적인 인과 효과를 한 쌍으로 확정하지 않는다.

LIO 최대 위치 오차는 high_speed 3.559m, dense 0.464m였다. 수신과 stamp는 계속 정상 관측되었지만 큰 위치 오차를 숨기거나 정확도가 해결된 설정으로 표현하지 않는다. 사용자의 우선순위에 따라 임의의 0.5m cutoff를 적용하지 않고, 정상 완주·직접 접촉·형상 여유와 함께 보고한다. 같은 YAML의 새 세션 반복 두 쌍을 추가하며, 반복 전 최종값으로 선정하지 않는다.

## 공통 실험 기준의 최종 반복·설치 결과

`radius050-time5e7`은 아래 새 GUI 6회 모두 원래 경유점/정지 조건으로 왕복·착륙했다. 외부 접촉 보고 0건, callback 오류/유실 0, 기록 종료 커버리지 유효였다. TRACKING 경계에서 floor found−lost pair 수가 0이고, 주행과 겹치는 floor 전환 구간도 없었다. LIO stamp 역행도 관측하지 않았다.

| 장면 | 주행 sim s (3회) | 실제 최고속도 m/s (3회) | 전체 CAD 여유 하한 m (3회) | LIO 최대 오차 m (3회) |
|---|---|---|---|---|
| high_speed | 44.725 / 44.055 / 48.100 | 7.655 / 7.323 / 7.338 | 0.808 / 1.279 / 0.691 | 3.559 / 0.544 / 0.555 |
| dense | 130.435 / 186.035 / 222.835 | 7.214 / 6.428 / 5.597 | 0.139 / 0.238 / ≥0.150 | 0.464 / 1.152 / 0.994 |

여기서 0.150m로 계산이 제한된 항목은 세밀 계산 cap까지의 하한이며 정확한 최솟값 0.150m를 뜻하지 않는다. 실제 acc/jerk 최대는 high_speed 7.683m/s²/31.622m/s³, dense 6.620m/s²/39.754m/s³였다. nominal 6/10 제한의 실제 준수를 달성한 설정은 아니다. 최대 관측 메시지 간격은 약 0.625초이며 마지막 TRACKING snapshot까지의 메시지 침묵 tail은 각 실행 약 0.03초였다. 메시지 사이 간격만 확인하면 마지막 메시지 뒤 침묵을 놓칠 수 있어 tail 및 fault 시각까지의 간격도 별도 기록한다.

`common_super_lio.yaml`을 위 검증 파일과 바이트까지 동일하게 갱신했다. SHA256은 `8dbc396dc28df0c9f51fceaa5635f305bc0ab01a3bf7dc410ffc57c140158ee0`이다. 이전 common 파일은 `common_super_lio_20hz_reference.yaml`에 보존했으며 이전 SHA256 `40ef48f821b1c22d405315c56ca1a56ede2e29b7fdf7a9095fb941ee574ed8ac`도 확인했다. zav_super 한 패키지만 빌드했고 install의 common 파일도 검증 SHA와 일치했다. 시나리오별 legacy YAML과 controller/SUPER/Super-LIO C++는 변경하지 않았다.

```bash
# 새 디렉터리/파일 경로로 진단을 켜면 검증과 같은 관측 조건을 사용한다.
ZAV_CONTACT_REPORT_PATH="$PWD/log/my_common_trial/contacts.jsonl" \
  ./run_super_sim.sh high_speed
# 다른 터미널
./run_super.sh --scenario high_speed --backend super_lio --profile common --execute
```

종료 후 양쪽 scenario만 dense로 바꾸면 같은 common YAML을 읽는다. contact 파일은 실행마다 새로운 경로를 지정한다. contact 환경변수를 생략하면 선택적 계측이 꺼진다. 위 6회는 선택적 계측을 활성화한 GUI 시험이며 계측 영향이 통계적으로 0이라고 보증하지 않는다.

이 설정은 **반복 검증한 공통 실험 기준**이다. 가장 빠른 공통 최적값, 실제 20m/s 달성 또는 혼합 맵 검증 완료를 뜻하지 않는다. dense 시간이 130~223초로 달라졌고 큰 LIO 최대 오차도 남아 있으므로 단발 최고속도만으로 개선을 확정하지 않는다. 예전 반경 0.45 후보는 dense가 더 빨랐던 실행도 있었지만 세 번째 여유 하한이 0.0665m로 좁았다. 기본값 채택은 더 넓은 여유를 확보한 실험 기준을 남기기 위한 선택이다.

## 기존 goal_vel_en 옵션의 기각

소스에서 goal_vel_en=false일 때 로컬 exp trajectory 끝 속도를 0으로 설정한다. true이면 충분히 먼 목표에서 global 목표 방향의 max_vel/2 끝 속도를 사용한다. 현재의 20m/s 상한에서는 10m/s 끝 속도가 된다. 이것을 별도 `goalvel` 후보에만 켜서 두 GUI 장면을 비교했으며 알고리즘이나 보호 코드는 수정하지 않았다.

high_speed는 실제 peak 9.618m/s를 관측했지만 주행 4.865초 뒤 heartbeat 보호로 미완주했다. 최대 실제 acc도 16.083m/s²로 컸다. 마지막 메시지부터 fault observer까지 약 1.035초였고 그동안 LIO 199개가 수신되었다(최대 수신 공백 약 41ms). 마지막 궤적은 이미 약 1.004초 전에 끝났으며 끝 속도는 10m/s, 실제 LIO 속도는 약 3.376m/s, endpoint 거리는 약 7.022m였다. 따라서 정상 terminal stop 유지 조건이 아니다. 앞선 Q600 후보의 긴 계산 callback 해석과 이 실패를 혼동하지 않는다. 관측은 heartbeat 발행 중단과 반복 최적화 위반 경고를 보여주며, 정확한 각 callback 실행 시간을 직접 계측한 것은 아니다.

dense는 PlanFromRest의 반복 최적화 위반으로 READY에서 출발하지 못했다. 40초 출발 대기 보호에 따른 착륙은 완주가 아니다. 이 실행도 evaluation과 비교 JSON의 출발 전 실패 목록에 보존했다. goal_vel_en=true는 공통 기본값에 반영하지 않았으며 현재 common은 false를 유지한다. 단순히 비정지 끝 속도를 강제하는 것만으로 두 환경에서 고속화되지 않았다. 후속 후보에서는 로컬 계획 거리·계산량·제동 가능 거리와 실제 제어 갱신을 함께 비교해야 한다.

## 25m 계획 거리 추가 비교

검증 기준에서 planning_horizon만 15→25m로 늘렸다. goal_vel_en=false, A* 창 [500,500,100], receding_dis=7.5m, 반경 0.50m, 모든 패널티/수렴 기준/제어/LIO와 GUI 조건은 동일하다. A* XY 반폭 37.5m 안의 값이며, 이전 Q300/r0.45 시험과 달리 현재 추종/여유 보강 후의 별도 비교다.

high_speed는 정상 완주·착륙, peak **10.779m/s**를 관측했다. 주행 시간은 44.110s로 기존 기준 44.055~48.100s 범위와 비슷했다. 실제 max acc/jerk=6.659/39.479, CAD 하한 0.482m, LIO 최대 오차는 **6.190m**였다. 높은 단발 peak만으로 더 좋은 공통 설정이라고 하지 않는다.

dense는 이동 후 첫 목표 근처에서 정지했지만, planner terminal endpoint가 `[0.525,50.175,2.325]`였다. 원래 경유점 `[0,50,1.5]`과의 거리는 **0.9934m**로 원래 허용 반경 0.8m 밖이다. 마지막 60 wall s 동안 LIO 기준 거리는 0.9976~1.0119m, 최대 속도 0.0295m/s로 대기 중이었다. 이를 완주로 바꾸거나 목표를 이동시켜 채점하지 않았다. 268.89 wall s 시점에 증거를 `operator-stop.json`으로 남기고 native land 서비스를 호출했다. 360초 timeout이 발생했다고 표현하지 않으며, 정상 착륙했어도 **임무 미완주**로 기록한다. 기록 중 TRACKING은 251.110sim s였고 peak 4.181m/s, max acc/jerk 5.579/41.884, CAD 하한 0.243m, LIO 최대 오차 1.509m였다. 외부 접촉 보고는 없었다.

끝점 대기의 발생을 planning_horizon 하나의 독립적인 인과 효과로 단정하지는 않는다. 실행마다 LIO·추정 지도·경로가 달랐고 더 긴 계획의 계산량도 달라진다. 그러나 이번 후보가 원래 임무 조건에서 완료하지 못했다는 사실은 유지한다. goal projection과 원래 경유점 조건의 관계는 이전에 사용자에게 확인했던 것과 같으며, 이번에도 0.8m를 유지했다. 25m 후보도 현재 common에 반영하지 않았다.

이번 캠페인은 GUI mission attempt **28회**를 포함한다. 호버 검증 및 contact 양성 fixture는 별도이며 이 수에 넣지 않는다. 기존 reference의 접촉, Q600 반복 실패, goalvel의 출발 전/주행 중 실패, H25의 목표점 대기도 삭제하지 않았다. 현재 기본 common은 6회 검증한 H15/r0.50/exp time5e7이며, 속도 최적화와 혼합 맵 검증은 아직 끝나지 않았다.
