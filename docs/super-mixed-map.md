# SUPER 혼합 맵과 공통 파라미터 주행

`mixed`는 개방→밀집→개방 구간을 한 장면에 합친 물리 시뮬레이션 맵이다. 기존 high_speed/dense 입력·YAML은 보존하며, 세 장면 모두 `src/zav_super/config/common_super_lio.yaml`을 명시해 같은 SUPER/OMMPC/Super-LIO 설정으로 주행할 수 있다. 환경별 파라미터 전환이나 별도 속도 제어 정책은 추가하지 않았다. zone 라벨, 원본 PCD와 ground truth는 planner/controller 입력으로 사용하지 않는다.

## 실행

워크스페이스 `/home/goat/zenith_ws`에서 실행한다. Isaac Sim의 `[Ready] mixed`가 출력된 뒤 두 번째 터미널을 시작한다. 기본 실행에서 Isaac Sim GUI와 RViz2가 모두 켜진다.

```bash
# 터미널 1
./run_super_sim.sh mixed
```

```bash
# 터미널 2
./run_super.sh --scenario mixed --backend super_lio --profile common --execute
```

임무는 `(0,-50,0.10)`에서 이륙해 `(0,50,1.5)`와 `(0,-50,1.5)`를 순서대로 방문한 뒤 착륙한다. 원래 경유점 0.8m 이내 및 추정 속도 0.7m/s 미만 조건을 유지한다. planner가 이동시킨 끝점 도달이나 수동 착륙만으로 완주로 세지 않는다. mixed는 common 프로파일/Super-LIO를 요구한다. 기존 시나리오별 legacy 기본 동작은 유지한다.

실행기는 저장 경로 `log/super_flights/<timestamp>/`를 출력한다. 착륙 후 해당 폴더에 `report.html`, 최대값을 표시한 `motion.png`, `summary.json`, `kinematics.csv`, `samples.jsonl`이 생성된다. navigation과 simulator는 각 실행 터미널에서 Ctrl+C로 종료한다. 다시 실행할 때는 이전 세션을 종료한 뒤 시작한다.

## 맵 구성

물리 영역은 기존과 같은 약 X 15m × Y 110m다. 왕복 이동 거리는 직선 기준 약 200m이며 실제 우회 경로는 더 길다.

| World y | 구간 |
|---|---|
| −55~−22m | 남쪽 개방 숲 |
| −22~−18m | 남쪽 연결 구간 |
| −18~18m | 중앙 밀집 숲 |
| 18~22m | 북쪽 연결 구간 |
| 22~55m | 북쪽 개방 숲 |

생성기는 기존 sparse 원본의 개방 구간과 dense 원본 y=[−40,−4]의 장애물 묶음을 사용한다. dense 묶음을 y축으로 +22m 이동해 중앙에 둔다. XY 0.2m 연결 성분 전체를 선택해, 경계에서 나무를 잘라 절단면을 만들지 않는다. 출발/회항/착륙 패드만 장애물 여유를 확보하고 중앙 직선 통로는 비우지 않았다. 실제 센서가 관측하는 3차원 숲이며, 비행 고도를 고정하거나 2차원 통로를 강제하지 않는다.

생성 파일은 `simulator/assets/super/mixed_sparse_dense.pcd`, 선택 이력은 `mixed_recipe.json`, 물리 장면은 `mixed.usdc`다. 원본 두 PCD의 SHA와 선택한 묶음 범위를 recipe에 보존한다. `pcd_scene.py`가 기존과 동일한 5cm 표면 voxel/연속 바닥으로 변환한다. PCD를 planner에 주입하지 않는다. 물리 200Hz·렌더 20Hz를 유지했다.

```bash
# 재생성/검사: Isaac 실행 없이 원본 PCD에서 생성할 수 있다.
python3 simulator/tools/super/mixed_scene.py
python3 simulator/tests/check_mixed_scene.py
```

## 접촉 계측과 구간별 결과 저장

아래 예시의 디렉터리는 매 실행 새 이름을 사용한다. contact 파일이 이미 존재하면 기록을 덮어쓰지 않는다.

```bash
# 터미널 1
ZAV_CONTACT_REPORT_PATH="$PWD/log/mixed_manual_01/contacts.jsonl" \
  ./run_super_sim.sh mixed
```

```bash
# 터미널 2
./run_super.sh --scenario mixed --backend super_lio --profile common --execute \
  --record-dir "$PWD/log/mixed_manual_01/telemetry"
```

```bash
# 주행·착륙 후 오프라인 분석
python3 simulator/tools/super/mixed_flight_report.py \
  log/mixed_manual_01/telemetry --contacts log/mixed_manual_01/contacts.jsonl
```

`mixed-report.html`, `mixed-flight.png`, `mixed-result.json`이 생성된다. 경로 색상은 실제 body 속도, 주황 음영은 실제 body가 중앙 밀집 영역에 있었던 시간이다. 개방/밀집 속도는 TRACKING 전체 시간 가중 평균으로, 끝점 부근 정지와 왕복을 포함한다. 지상 진단용 ground truth나 구간 정보가 제어에 되먹임되지 않는다.

native PhysX 장애물/미분류 외부 contact 보고를 착지 바닥 보고와 구분한다. 보고 오류·유실, 바닥 양성 확인 및 기록 커버리지를 확인하며, 계측하지 않은 실행을 접촉 0건으로 표시하지 않는다. acc/jerk는 약0.21초 평활 미분값으로 짧은 충격의 원시 최대값을 대신하지 않는다. CAD 거리 하한도 보간한 기록/형상에 대한 진단이며 native contact 측정과 구분한다.

## 실제 GUI 검증

검증 폴더는 `log/super_flights/mixed-20261116/`다. 11월16일 시작한 캠페인을 같은 폴더에 이어 기록했다. 실제 실행 결과는 아래에 집계한다. 모든 반복에서 공통 YAML과 맵의 SHA가 같은지 확인하며, 실패도 보존한다.

### 3회 결과

Isaac GUI + RViz2, 물리200Hz/렌더20Hz, Super-LIO만 사용했다. 모든 실행의 common YAML SHA256은 `8dbc396dc28df0c9f51fceaa5635f305bc0ab01a3bf7dc410ffc57c140158ee0`, 맵 SHA256은 `3322b8b6bbf3b7201e38a769855e5054fac963e62bb7d5f9862cf87146066491`로 같다. 기존 common YAML을 조정하지 않았고, 원본 두 맵과 legacy YAML도 시작 전 해시와 동일하다.

| 실행 | 결과 | 주행 sim s | 실제 max vel / acc / jerk | CAD 여유 하한 m | LIO 최대 오차 m |
|---|---|---:|---:|---:|---:|
| trial01 | 무접촉 왕복·착륙 | 82.46 | 8.49 / 7.01 / 37.91 | 0.251 | 0.673 |
| trial02 | 무접촉 왕복·착륙 | 89.89 | 8.55 / 7.21 / 31.27 | 0.194 | 1.211 |
| trial03 | 접촉 후 미완주·중단 착륙 | 393.15 (중단 전 부분값) | 7.18 / 15.06 / 170.32 | −0.014 | 0.868 |

vel/acc/jerk 단위는 m/s, m/s², m/s³다. 실패 실행의 peak를 완주 성능으로 집계하지 않는다. 음의 CAD 하한만으로 침투를 확정하지 않으며, 실패 실행에서는 별도로 PhysX impulse 응답도 확인했다. nominal max_vel=20, max_acc=6, max_jerk=10을 실제 기체가 엄격하게 준수했다는 결과는 아니다.

| 실행 | 남쪽 개방 평균 m/s | 중앙 밀집 평균 m/s | 북쪽 개방 평균 m/s |
|---|---:|---:|---:|
| trial01 | 4.22 | 1.73 | 4.81 |
| trial02 | 4.62 | 1.68 | 4.93 |
| trial03 | 4.47 | 0.60 | 4.69 |

정상 완주 두 실행에서는 밀집 구간의 평균 속도가 낮고 개방 구간에서 속도가 높았다. 이는 해당 경로·시점의 관측이며 별도 전환 로직을 추가한 결과가 아니다. trial03은 장시간 정체도 평균에 포함하므로 성공 실행과 구분한다.

세 번째 실행은 약141.685~141.775sim s에 0이 아닌 impulse 보고2개가 있었다. 0.25초 간격으로 묶으면 접촉 응답1구간이며, 근접/유지/해제 보고 수 전체를 충돌 횟수로 세지 않는다. 이후 A*의0.1초 탐색 제한 초과와 PlanFromRest 실패가 반복되어 밀집 구간에서 진행하지 못했다. 로그상의 순서이며 접촉이 이후 모든 탐색 실패의 독립적인 원인이라고 확정하지 않는다.415.10wall s에 실패 증거와 당시 상태를 저장하고 native LAND를 요청했다.600초 timeout으로 종료했다고 표현하지 않으며 착륙을 완료해도 왕복 완주는 아니다.

세 실행 모두 LIO stamp 역행0, 최대 stamp 간격은 약30ms였다. 첫 두 실행 LIO 최대/RMS 오차는0.673/0.212m,1.211/0.365m였고 실패 실행은0.868/0.229m였다. 오차가0.5m를 넘었다는 이유로 임의 탈락시키지 않았으며, 주행·접촉·정체와 함께 보고한다. 실제 수신 지연/명령 갱신 분포는 캠페인의 summary.json에 있다.

현재 성적은 **2/3 무접촉 완주**다. 맵과 실행 기능은 동작하지만 이 공통 파라미터의 혼합 맵 안정화 또는 최고속도 최적화가 완료된 것은 아니다. 세 번째 실패를 삭제하거나 실행별로 맵/파라미터를 변경하지 않았다.

전체 비교는 `log/super_flights/mixed-20261116/summary.md`와 `summary.json`, 실행별 보고서는 `trial01|02|03/mixed/mixed-report.html`에 있다. 각 보고서에는 맵·실제 경로·속도 색상, 밀집 구간 체류 음영, 속도·가속도·저크 최대값과 원본 JSON 링크가 포함된다. 실패 실행의 `operator-stop.json`, `contacts.jsonl`, `navigation.log`, 지도 스냅샷과 원본 odometry도 보존했다.

맵 생성 검사는 밀도 차이, 빈 중앙 직선 통로를 만들지 않았는지, 실제5cm voxel 기준 양 끝 패드 여유를 확인한다. 실제 XY voxel 여유는 약3.543/3.795m다. 원래 경유점 순서, 이동 중 통과/수동 착륙/shifted goal을 완주로 오인하지 않는 회귀 검사는 다음과 같다.

```bash
python3 simulator/tests/test_mixed_flight_report.py
python3 simulator/tests/check_sim_session.py
```

새 기록에는 맵 recipe/geometry 정보를 metadata에 함께 저장한다. 과거 보고서의 맵 SHA와 현재 자산이 다르면 재분석 시 다른 맵으로 덮어쓰지 않는다. 이미 생성된 HTML/PNG는 해당 실행의 결과를 그대로 보존한다.
