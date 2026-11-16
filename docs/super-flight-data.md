# SUPER 주행 데이터 보기

기존 실행 명령에서 자동으로 기록한다. 제어·플래너 파라미터는 바꾸지 않는다.

```bash
./run_super_sim.sh high_speed
# 다른 터미널
./run_super.sh --scenario high_speed --backend super_lio --execute
```

착륙 후 보고서가 자동 생성된다. 중간에 종료하려면 시뮬레이터보다 navigation 터미널을 먼저 `Ctrl+C`로 종료하면 수집된 구간을 분석한다. 보고서 생성은 비행이 끝난 뒤 수행한다.

```bash
./show_super_flight.sh
```

이 명령은 가장 최근 기록을 브라우저로 연다. 결과 경로만 출력하려면 `--no-open`을 붙인다. 보고서의 기본 최댓값은 `TRACKING` 구간이며 이륙·착륙 포함 전체 기록은 펼쳐서 별도로 확인할 수 있다. 실제 기체 데이터가 없으면 LIO 추정값을 사용했다고 명시한다.

- `report.html`: 최대 속도·가속도·저크 카드, 피크 시각, 그래프. 이미지가 포함되어 HTML 한 파일만으로 그래프를 볼 수 있다. CSV 링크는 옆 파일이 필요하다.
- `motion.png`: 주행 구간 그래프. 실제 기체와 LIO 추정값, 실제 기체 기준 최대값 마커, 설정 boundary 점선.
- `full_flight.png`: 이륙·착륙 포함 그래프. 착륙 접촉 피크도 포함하므로 주행 피크와 구분한다.
- `summary.json`: 소스별/구간별 최댓값과 시각, 미분 방법.
- `kinematics.csv`: 시간, 상태, world ENU의 vx/vy/vz·ax/ay/az·jx/jy/jz 및 각 벡터 크기.
- `samples.jsonl`: 수신한 원본 odometry의 pose·twist 및 controller 상태. `metadata.json`에는 실행 설정·장착 보정을 저장한다.

기본 폴더는 `log/super_flights/YYYYMMDD-HHMMSS-ffffff/`이며 터미널에도 경로를 출력한다. 실행마다 새 폴더를 생성한다. 특정 실행을 지정하려면:

```bash
./run_super.sh --scenario high_speed --backend super_lio --execute \
  --record-dir log/super_flights/my-run-01  # 아직 없는 새 폴더
./show_super_flight.sh log/super_flights/my-run-01
```

## 계산 기준

기본 수치는 **3차원 벡터 크기**다. 가속도는 속도 벡터의 시간 미분이며 중력을 포함한 IMU specific force가 아니다. 따라서 속력의 크기가 일정해도 회전하면 가속도와 저크가 나타난다.

실제 기체 데이터는 `/drone0/ground_truth/odometry`의 mounted IMU 속도를 world ENU로 회전하고 `ω × lever-arm` 속도를 빼서 body 원점 속도로 변환한다. LIO는 `/zav/super/odom`의 world 속도를 사용한다. 이 노드는 구독만 하며 ground truth를 제어에 주입하지 않는다.

원래 odometry timestamp(시뮬레이션 초)로 각 연속 구간을 균일하게 재표본화한다. 속도 벡터에 3차 Savitzky–Golay 필터를 적용해 1차/2차 미분으로 가속도/저크를 계산한다. 기본 윈도는 약 **0.21초**다. 잡음이 미분에서 커지므로 이 값은 필터 조건을 명시한 최대값이며, 무필터 순간 접촉 피크와 같지 않다. 각 연속 구간 양끝 반 윈도는 제외하고, 0.1초 초과 수신 공백·시간 역행·비정상 샘플은 구간을 나눠 미분한다.

다른 필터 폭으로 재분석하려면 원본 기록에서 다시 계산한다:

```bash
./show_super_flight.sh log/super_flights/my-run-01 --window 0.11
# 브라우저 없이 파일만 생성
python3 simulator/tools/super/flight_metrics.py log/super_flights/my-run-01 --window 0.21
```

점선은 설정 boundary이며 실제 움직임의 엄격한 상한이 아니다. 특히 현재 exp/backup의 `penna_jerk`가 음수이면 max_jerk 벌점이 꺼져 있으므로 해당 점선은 표시하지 않는다. 플래너가 지정한 max_jerk와 관측된 최대 jerk를 혼동하지 않는다.

직접 `ros2 launch`를 사용할 때에는 `telemetry_dir:=/절대경로/새폴더`를 넘겨 기록을 켤 수 있다. 기본 `run_super.sh`는 이 인자를 자동 지정한다.
