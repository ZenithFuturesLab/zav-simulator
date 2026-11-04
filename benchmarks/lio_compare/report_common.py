#!/usr/bin/env python3
from pathlib import Path
import json
import pandas as pd
R=Path(__file__).resolve().parents[2]/'reports/lio-common-20261102'
s=pd.read_csv(R/'comparison.csv');a=s.set_index('algorithm');names=['Super-LIO','FAST-LIVO2 (LIO only)']
assert all(a.loc[n,'trials']==3 for n in names)
rows=[('처리 평균 ± 회차 표준편차 (ms)','total_ms',1),('처리 P95 (ms)','p95_ms',1),('프로세스 CPU (%)','cpu_percent',1),('최대 RSS (MiB)','peak_rss_mib',1),('처리 점 수/스캔','selected_points',1),('IMU 전파 위치 RMSE (cm)','imu_ate_rmse_m',100),('IMU 전파 위치 P95 (cm)','imu_ate_p95_m',100),('IMU 전파 최대 위치 오차 (cm)','imu_ate_max_m',100),('IMU 전파 자세 RMSE (deg)','imu_rotation_rmse_deg',1),('LiDAR 보정 위치 RMSE (cm)','lidar_ate_rmse_m',100)]
table='| 항목 | Super-LIO | FAST-LIVO2 LIO |\n|---|---:|---:|\n'
for label,key,scale in rows:
 values=[f'{a.loc[n,key]*scale:.2f}'+(f" ± {a.loc[n,'total_ms_std']:.2f}" if key=='total_ms' else '') for n in names]
 table+='| '+label+' | '+' | '.join(values)+' |\n'
f=a.loc[names[1],'total_ms']/a.loc[names[0],'total_ms'];window=json.loads((R/'evaluation_window.json').read_text())
(R/'REPORT.md').write_text(f'''# 동일 포인트·GT 한 바퀴 재비교

동일한 실제 Isaac Sim 비행 기록 중 첫 정사각형 한 바퀴만 평가했다. 평가 시각 51.01~80.75초, GT 경로 길이 {window['path_length_m']:.2f}m. 초기화·지도 구축 이력을 같게 유지하기 위해 기록 시작부터 재생했고, 정지 구간 5~15초로 좌표를 정렬했다. GT를 알고리즘 입력으로 사용하지 않았다.

{table}

각 알고리즘 3회 독립 프로세스 재시작의 평균이다. 반복 비행 3회가 아니라 동일 데이터 재생 3회다. CPU 100%는 논리 코어 하나 기준이고 최대 RSS와 P95도 각 회차 값의 평균이다. 평균 스캔 처리 시간 비율은 {f:.2f}배다. 이전 결과와는 평가 구간·전처리 조건이 다르므로 정확도 수치를 직접 이어 비교하면 안 된다.

![연산 비교](compute.png)
![GT와 추정 궤적·오차](accuracy.png)

## 같은 포인트를 보장한 방법

1. GT를 보지 않고 각 raw LiDAR 스캔에 공통 0.5m 복셀 필터를 적용했다. 복셀별 첫 실제 반환점을 택하고, 스캔 시작·종료 시각 유지를 위해 유효한 첫·끝 반환점도 유지했다. 평균점으로 새 좌표를 생성하지 않았다.
2. 거리 0.5~100m, 유효 tag·line 조건을 공통 적용하고, 선택된 반환점의 XYZ·반사강도·개별 측정 시각을 그대로 보존했다. 동일 직렬화 메시지를 두 알고리즘에 재생했다.
3. 두 알고리즘의 입력 간격 필터를 1로 설정하고 자체 다운샘플은 벤치마크 복사본에서 우회했다. 최대 반복 횟수 4회, 가속도 잡음 설정 0.1, 스레드 최대 4개를 맞췄다.
4. FAST-LIVO2의 조밀한 원본 스캔용 point-time 간격 휴리스틱도 공통 입력 시험에 한해 우회했다. 희소 선택 후에도 원래 포인트 시각이 변경되지 않게 하기 위한 조치다.
5. 평가 구간 297개 스캔 각각에서 두 알고리즘·전체 6회차의 입력 점 수와 추정에 전달되는 점 수가 공통 선택 결과와 일치함을 검증했다. 점 개수 평균은 약 5,779.49개이며 스캔마다 동일 개수로 강제한 것이 아니라 동일 스캔의 동일 반환점을 사용했다. `point_verification.json`과 `common_point_manifest.json`에 검증 및 해시를 기록했다.

공통 선택은 raw LiDAR 좌표에서 한 번 수행했다. 각 알고리즘의 IMU 적분·모션 왜곡 보정·지도·추정기는 그대로 유지했다. 따라서 같은 원본 점이라도 추정 상태에 따라 왜곡 보정 후 XYZ는 달라질 수 있다. 보정 후 좌표까지 외부에서 같게 덮어쓰는 실험은 아니다. 공통 전처리 시간은 mapper 시간·CPU에서 제외했다. 스캔 계측에는 native IMU/왜곡 보정, 공통 점 복사, 대응점 처리·추정, 지도 갱신, 출력이 포함된다.

## GT와 오차 계산

GT는 물리 200Hz마다 PhysX rigid-body pose를 직접 읽고 장착 변환을 적용한 IMU pose다. 그래프의 검은 점선은 첫 바퀴의 GT 하나이며 반복 경로를 겹쳐 표시하지 않았다. 기존 기록을 재사용했으며 새 비행은 수행하지 않았다.

초기 정지 pose만으로 고정 rigid 좌표 정렬했다. 전체 경로 fitting, 스케일 fitting, 시간 지연 최적화는 하지 않았다. 위치 선형 보간과 자세 SLERP로 공통 10Hz 시각에서 평가했고, 모든 알고리즘·회차의 유효 시각 교집합을 사용했다. IMU 전파 odometry와 LiDAR 보정 odometry는 별도로 집계했다. IMU 전파는 각 측정 시각의 최초 발행값을 썼다. 오차 곡선은 대표 1회, 표는 3회 평균이다. 유효 비율은 comparison.csv의 coverage 항목에 있다.

## 연산 계측 범위

현재 ZAV Super-LIO(IMU 재전파 수정 포함)와 FAST-LIVO2 카메라 OFF LIO 모드의 비교다. FAST-LIO2 또는 논문 baseline 재현 실험이 아니다. Ryzen 5 7600 / WSL2, mapper affinity 논리 CPU 0~3, -O3 -DNDEBUG, PCD 저장 OFF, 센서 시각 1배속 재생이다. mapper 계측 동안 Isaac Sim·PX4·RViz는 실행하지 않았다. 물리 200Hz 제품 설정 및 제품 코드는 변경하지 않았다.

대응점 처리와 상태 추정은 합산 구간이므로 KNN·행렬 해법 비용을 각각 분리한 값은 아니다. 프로세스 전체 CPU에는 고주파 IMU 예측, ROS 입력·executor 등이 포함된다. FAST-LIVO2는 카메라 OFF에서도 VIO 관련 초기화가 있어 RSS 차이를 지도 구조만으로 설명할 수 없다.

이 결과는 단일 시뮬레이션 환경의 한 바퀴에 한정된다. 실제 드론 정확도 또는 시뮬레이터 FPS 향상 배율을 보장하지 않는다.

[PDF](comparison_report.pdf) · [집계 CSV](comparison.csv) · [단계별 CSV](stage_comparison.csv) · [전체 회차 CSV](all_trials.csv)
''')
print(table)
