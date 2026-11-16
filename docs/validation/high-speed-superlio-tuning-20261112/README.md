# high_speed + Super-LIO GUI 속도 튜닝 — 2026-11-12

최종 검증: **PASS**, 새 Isaac Sim GUI + RViz2 세션 3회. 최종 설정은 `candidate-v55-a2`의 YAML을 사용했다. 모든 연속 파라미터 공간의 전역 최적값을 증명한 결과가 아니라, 이 머신·맵·왕복 임무에서 시험한 값 중 3회 연속 검증을 통과한 가장 빠른 설정이다.

```yaml
boundary:
  max_vel: 5.5
  max_acc: 2.0
  max_jerk: 8.0
  max_omg: 1.2
  max_acc_thr: 20
  min_acc_thr: 6.0
  penna_margin: 0.05
```

기준 4/2/8/1.2의 왕복 추종 구간은 57.19 sim s / 59.76 wall s. 최종 반복은 45.00–45.50 sim s. 기준 대비 시간 단축 범위는 20.4–21.3%. 이륙·착륙 시간을 제외한 실제 TRACKING 구간이며 정지와 회항 대기는 포함한다.

## 적용과 재실행

`src/zav_super/config/high_speed_super_lio.yaml`은 `high_speed + super_lio`에만 선택된다. launch가 실제 설정 경로를 출력하며, 기존 실행 명령은 그대로다. source와 install의 최종 전용 YAML/launch를 동기화했다. dense·다른 LIO 설정, OMMPC, LIO 추정기 설정, 안전 임계값은 수정하지 않았다. 탐색 중 변경한 계획 반경/팽창은 최종 선택 YAML을 기준으로 확인한다.

```bash
./run_super_sim.sh high_speed
# 다른 터미널
./run_super.sh --scenario high_speed --backend super_lio --execute
```

최종 계획 반경: 0.33m, inflation_step: 3 (resolution 0.15m).

## 독립 검증 기준

- 왕복 두 목표를 실제 물리 위치로 각각 0.8m 이내 통과하고 정상 착륙·disarm, FAULT/STOPPED 없음.
- 이륙부터 착륙까지 timestamp를 맞춘 Super-LIO body 위치 오차: 최대 0.5m 이하, RMS 0.25m 이하.
- 추종 중 odometry stamp 최대 공백 50ms 이하, 벽시계 수신 최대 공백 200ms 이하.
- 실제 body 중심 경로와 장애물의 보수적 여유 0.1m 이상. body 외접구 반경 0.33m, USD 복셀 재구성 오차 상한 0.086603m, 1cm 이하 선분 표본의 반간격 0.005m를 최근접 PCD 거리에서 차감했다. 지면은 이 장애물 거리 검사에서 제외한다. 이는 기록된 물리 위치 사이 선분에 대한 검사이며 별도의 PhysX contact counter는 아니다.
- Ground truth는 이 진단과 시험 종료 판단에만 사용하며, 추정·계획·제어에는 주입하지 않았다.

## 최종 반복

| 실행 | sim s | wall s | 실제 최고 m/s | 전체 LIO 최대 m | 전체 LIO RMS m | 기체 여유 하한 m | 최대 수신 공백 ms | 판정 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| [final-v55-repeat-1](final-v55-repeat-1/metrics.json) | 45.50 | 47.53 | 5.711 | 0.397 | 0.170 | 0.465 | 42.70 | PASS |
| [final-v55-repeat-2](final-v55-repeat-2/metrics.json) | 45.30 | 47.31 | 5.679 | 0.365 | 0.143 | 0.501 | 47.21 | PASS |
| [final-v55-repeat-3](final-v55-repeat-3/metrics.json) | 45.00 | 46.99 | 5.732 | 0.379 | 0.154 | 0.280 | 44.66 | PASS |

## 탐색 기록

| 실행 | v / a / j / omg | 추종 sim s | 추종 LIO 최대 m | 여유 하한 m | 판정 |
|---|---|---:|---:|---:|---|
| [v25-a2-installed](v25-a2-installed/metrics.json) | 25.0 / 2.0 / 8.0 / 1.2 | 34.00 | 0.629 | 0.019 | FAIL |
| [baseline-v4-a2](baseline-v4-a2/metrics.json) | 4 / 2 / 8 / 1.2 | 57.19 | 0.210 | 0.548 | PASS |
| [v12-a4](v12-a4/metrics.json) | 12 / 4 / 20 / 1.2 | 26.20 | 61.920 | 0.837 | FAIL |
| [v8-a3](v8-a3/metrics.json) | 8 / 3 / 16 / 1.2 | 31.55 | 2.203 | 0.075 | FAIL |
| [v10-a2](v10-a2/metrics.json) | 10 / 2 / 8 / 1.2 | 28.07 | 2.166 | 0.197 | FAIL |
| [v10-a3](v10-a3/metrics.json) | 10 / 3 / 16 / 1.2 | 23.80 | 2.765 | -0.153 | FAIL |
| [v8-a2](v8-a2/metrics.json) | 8 / 2 / 8 / 1.2 | 36.20 | 0.630 | 0.317 | FAIL |
| [v6-a3](v6-a3/metrics.json) | 6 / 3 / 16 / 1.2 | 40.80 | 0.536 | 0.806 | FAIL |
| [v6-a2](v6-a2/metrics.json) | 6 / 2 / 8 / 1.2 | 42.60 | 0.348 | 0.149 | PASS |
| [v8-a3-w05](v8-a3-w05/metrics.json) | 8 / 3 / 8 / 0.5 | 33.60 | 0.508 | 0.436 | FAIL |
| [v10-a3-w05](v10-a3-w05/metrics.json) | 10 / 3 / 8 / 0.5 | 24.70 | 2.366 | 0.246 | FAIL |
| [v7-a25-w05](v7-a25-w05/metrics.json) | 7 / 2.5 / 8 / 0.5 | 41.80 | 0.981 | 0.069 | FAIL |
| [v75-a25-w05](v75-a25-w05/metrics.json) | 7.5 / 2.5 / 8 / 0.5 | 35.70 | 0.430 | -0.049 | FAIL |
| [v8-a25-w03](v8-a25-w03/metrics.json) | 8 / 2.5 / 8 / 0.3 | 33.83 | 2.225 | 0.429 | FAIL |
| [v7-a2](v7-a2/metrics.json) | 7 / 2 / 8 / 1.2 | 20.50 | 0.505 | 0.679 | FAIL |
| [v8-a15-w05](v8-a15-w05/metrics.json) | 8 / 1.5 / 8 / 0.5 | 40.37 | 2.094 | -0.100 | FAIL |
| [v75-a25-w05-clearance](v75-a25-w05-clearance/metrics.json) | 7.5 / 2.5 / 8 / 0.5 | 21.50 | 0.517 | 0.262 | FAIL |
| [v8-a25-w05-clearance](v8-a25-w05-clearance/metrics.json) | 8 / 2.5 / 8 / 0.5 | 22.00 | 1.140 | 0.309 | FAIL |
| [final-v6-repeat-1](final-v6-repeat-1/metrics.json) | 6 / 2 / 8 / 1.2 | 42.80 | 0.269 | 0.343 | PASS |
| [final-v6-repeat-2](final-v6-repeat-2/metrics.json) | 6 / 2 / 8 / 1.2 | 42.70 | 0.415 | 0.435 | PASS |
| [final-v6-repeat-3](final-v6-repeat-3/metrics.json) | 6 / 2 / 8 / 1.2 | 36.60 | 2.121 | 0.294 | FAIL |
| [final-v55-repeat-1](final-v55-repeat-1/metrics.json) | 5.5 / 2.0 / 8.0 / 1.2 | 45.50 | 0.397 | 0.465 | PASS |
| [final-v55-repeat-2](final-v55-repeat-2/metrics.json) | 5.5 / 2.0 / 8.0 / 1.2 | 45.30 | 0.365 | 0.501 | PASS |
| [final-v55-repeat-3](final-v55-repeat-3/metrics.json) | 5.5 / 2.0 / 8.0 / 1.2 | 45.00 | 0.379 | 0.280 | PASS |

음수 여유는 보수적인 구형 외접체로 무접촉을 입증하지 못했다는 의미다. 실제 접촉을 직접 계수한 결과로 해석하지 않는다.

초기 `v25-a2-installed`는 소스 max_vel=4와 설치본 max_vel=25의 불일치를 발견한 실행이다. 이 실행은 10Hz snapshot만 남아 수신 공백의 정밀 검증 대상에서 제외했다. 설치본 원본은 이 폴더의 YAML에 보존했고, 다른 backend가 사용하는 설치본은 원래 바이트로 복원했다. `v4-a2`는 초기 하네스가 이전 simulator에 attach한 데다 recorder 오류가 발생한 무효 시험으로 비교에서 제외했다. 이후 모든 trial은 새 simulator를 시작했고 recorder 정상 실행을 검사했다.

`max_jerk`는 exp/backup의 `penna_jerk`가 음수이므로 현재 강제되는 제한이 아니다. 코드의 `weightJer > 0` 분기에서만 벌점이 켜진다. 최대 jerk 숫자 변경만으로 성능 향상을 주장하지 않았으며 실제 reference jerk는 각 metrics에 별도로 남겼다. 최종 선택에서 이 weight는 변경하지 않았다.

수신 약 200Hz만으로 위치 정확도를 판정할 수 없었다. 8~12m/s 일부 시험은 수신 공백 없이도 LIO가 수 m 이상 틀어졌고, 일부는 추정 좌표상 왕복 완료를 출력했다. 물리 위치와 비교해 FAIL 처리했다. 회항 시 새 계획 생성 지연으로 heartbeat 보호 착륙한 실행도 FAIL 처리했으며 보호 제한을 완화하지 않았다.

6m/s·가속도 2 조합은 탐색 및 최종 반복 첫 두 번을 통과했지만 세 번째 반복에서 LIO 오차 2.121m로 중단했다. 따라서 개별 PASS가 있어도 최종 추천에서는 제외했다. `final-v6-verification.json`에 이 실패를 보존했다.

## 기록과 도구

- [전체 숫자](summary.json), [환경·파일 해시·판정 기준](environment.json), [비교 그래프](comparison.png)
- 각 trial: loaded-high_speed.yaml, flight.jsonl, odom-events.jsonl, metrics.json, GUI 화면·window tree. 로그는 로컬 launch.log/isaac.log이며 저장소의 *.log 제외 규칙을 따른다.
- `run_trial.py`: 새 GUI 시뮬레이터, RViz 포함 동일 사용자 명령, 개별 수신 기록. 종료 시 자신이 시작한 프로세스 정리 및 설치 YAML 복원.
- `analyze.py`: 물리 위치/odom 시각 정렬·거리·수신 품질 계산. `verify_selected.py`: backend 전용 설정 적용 후 새 세션 3회 검증.
- [최종 GUI 화면](final-v55-repeat-3/gui-midflight.png).
