#!/usr/bin/env python3
from pathlib import Path
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[2];O=R/'reports/lio-spike-diagnosis-20261102';d=pd.read_csv(O/'diagnostic_comparison.csv')
# Import only data preparation and trajectory evaluation; not the graph/report side effects.
s=(R/'benchmarks/lio_compare/analyze_diagnostic.py').read_text();exec(s[:s.index('fig,axes=')]);d=pd.read_csv(O/'diagnostic_comparison.csv')
fig,axes=plt.subplots(2,1,figsize=(12,7),constrained_layout=True)
for ax,alg,name in zip(axes,['super_lio','fast_livo'],['Super-LIO','FAST-LIVO2 (camera OFF)']):
 for group,label,color in [('raw','Original per-point timing','#b94033'),('snapshot-time','Render-frame geometry timing','#16776a')]:
  tt,ep,er=series[(group,alg,'imu')];ax.plot(tt,100*ep,label=label,color=color)
 ax.set(title=name,xlabel='Simulation time [s]',ylabel='IMU odometry position error [cm]',xlim=(65.5,75));ax.legend();ax.grid(alpha=.25)
fig.suptitle('Timing diagnosis: identical point XYZ and original reception schedule\nOnly measurement-time interpretation changed; no GT pose fed to either estimator',fontsize=11);fig.savefig(O/'timing_intervention.png',dpi=160);fig.savefig(O/'timing_intervention.pdf');plt.close(fig)
table='| 알고리즘·조건 | IMU 위치 RMSE cm | 최대 위치 오차 cm | 최대 5ms 위치 변화 cm | LiDAR 위치 RMSE cm |\n|---|---:|---:|---:|---:|\n'
for alg in ['super_lio','fast_livo']:
 for group in ['raw','snapshot-time']:
  z=d[(d.algorithm==alg)&(d.group==group)&(d.kind=='imu')].iloc[0];l=d[(d.algorithm==alg)&(d.group==group)&(d.kind=='lidar')].iloc[0]
  table+=f"| {alg} / {group} | {z.rmse_cm:.2f} | {z.max_cm:.2f} | {z.max_5ms_position_change_cm:.2f} | {l.rmse_cm:.2f} |\n"
(O/'REPORT.md').write_text(f'''# Odom 급기동 점프 재현·원인 분리

2026-11-02. 기존 실제 Isaac Sim 비행의 동일 센서 기록을 재생했다. 새 비행을 수행한 결과가 아니다. FAST는 FAST-LIVO2 카메라 OFF 모드다. 각 진단 조건 1회이며 성능 벤치마크가 아니다. 평가 구간은 기존 첫 바퀴 51.01~80.75초이고 초기 정지 5~15초의 고정 좌표 정렬만 사용했다.

## 확인한 주원인: LiDAR 좌표와 측정 시각의 불일치

같은 XYZ를 개별 timestamp의 PhysX 자세로 월드에 배치한 경우보다, 해당 렌더 프레임 시작 자세로 배치한 경우 정지 지도에 더 잘 맞았다. 급기동 65.7~67.5초의 point-to-plane RMSE는 12.27→3.90cm, 73.25~74.6초는 15.76→4.29cm였다. 같은 후보 반환점 집합과 같은 정지 지도·평면을 비교한 센서 일관성 진단이다. 후보 평면·가시성 한계가 있어 절대 센서 정확도로 해석하지 않는다.

기존 Livox 변환은 GPU의 점별 timestamp를 보존하지만 XYZ가 어느 자세를 기준으로 하는지 별도로 검증하지 않았다. LIO는 이 timestamp를 이용해 IMU 모션 보정을 수행하므로, 좌표가 나타내는 자세와 timestamp가 불일치하면 급회전 때 보정된 스캔이 지도와 어긋난다. 이어지는 LiDAR 보정이 큰 상태 수정을 만들고, 고주파 출력도 그 상태를 기준으로 재전파하면서 점프한다.

원본 재생의 73.7998초에서 Super-LIO LiDAR 위치 수정 18.04cm, 속도 수정 0.864m/s, 동일 고주파 시각에서 재전파 위치 수정 18.51cm를 기록했다. 별도 기존 출력에서도 73.800→73.805초의 실제 GT 이동은 약 0.67cm, 추정 위치 변화는 약 18.47cm였다. 실제 기체가 그만큼 이동한 현상은 아니다.

## 직접 개입 시험

같은 원본 XYZ·반사강도·점 ID를 유지하고, 점의 측정 시각을 기록된 렌더 시작 시각으로 해석하도록 바꿨다. 원래 메시지 도착 스케줄도 유지했다. 이 입력 변환은 GT pose를 읽지 않으며, 추정기에 GT 위치·자세를 주지 않았다. 렌더 주기 50ms·기록된 위상 10ms에 의존하는 이번 데이터 전용 진단이다. 실시간 제품 수정으로 그대로 적용하지 않았다.

{table}

![원본과 시각 해석 변경](timing_intervention.png)

Super-LIO에서 큰 감소가 나타나 시간·좌표 불일치가 주요 원인임을 뒷받침한다. FAST-LIVO2에도 개선이 있지만 잔여 점프가 남아 모든 원인이 제거됐다고 볼 수 없다. 위 결과는 진단 입력에 대한 수치이며, 원래 알고리즘 간 실환경 정확도 비교나 새 비행 정확도로 주장하지 않는다.

## 별도로 확인한 문제와 대조군

- **IMU 자세 지연**: Vehicle.update_state가 위치·속도는 PhysX에서, 자세는 렌더용 USD에서 읽었다. 캡처 state 자세를 직접 PhysX 자세와 대조하면 약 50ms 지연, 평가 구간 최대 약 9.1° 차이가 있었다. IMU 좌표 변환 및 레버암 가속도 계산에 서로 다른 시각의 상태가 섞인다.
- 이 불일치만 GT 보조로 역변환한 진단은 Super IMU RMSE 4.41→4.31cm, 최대 5ms 변화 18.47→17.65cm였다. 실제 버그지만 큰 점프의 주원인으로 설명하기에는 부족했다. FAST RMSE는 4.10→3.61cm였다. 이 역변환 대조군은 GT-assisted이며 실제 배포 알고리즘이 아니다.
- **스캔 끝 IMU 누락**: Super sync가 스캔 끝 이전 IMU만 모아 보정 상태 시각이 스캔 끝보다 약 4.8ms 앞섰다. 끝 시각 보간을 추가한 벤치마크 대조군은 RMSE 4.41→4.17cm, 최대 5ms 변화 18.47→12.61cm였다. 이 대조군 변경은 제품에 적용하지 않았다.
- 모션 보정 OFF: Super RMSE 13.01cm로 악화. 따라서 단순히 deskew를 꺼서 해결할 문제가 아니다.
- GT-assisted 이상적 deskew만 적용: Super RMSE 약 4.11cm, 최대 변화 약 17.93cm. 잘못된 시각·좌표 해석을 그대로 두면 참 궤적으로 보정해도 큰 점프가 남았다.

## 제품 변경 및 남은 검증

Vehicle.update_state는 자세도 PhysX pose에서 읽도록 수정했다. 회귀 테스트는 기존 코드에서 실패하고 변경 후 통과했으며, mounted IMU 수치 테스트도 통과했다. 물리 설정은 200Hz 그대로다. Super-LIO 경계 보간, deskew OFF, GT 보조 역변환 등은 벤치마크 복사본에만 적용했다.

LiDAR 제품 경로는 아직 변경하지 않았다. 실행 시 GMO frameStart/frameEnd pose·motionCompensationState·Motion BVH 상태를 직접 기록해 좌표 기준을 명시해야 한다. Isaac Sim 5.1 공식 문서는 RTX LiDAR 모션 효과의 정확한 모델링에 Motion BVH가 필요하며 기본값은 OFF라고 설명한다. 현재 SimulationApp 생성에는 명시적 활성화가 없다. 기록 당시 런타임 설정 덤프가 없으므로 OFF 여부를 직접 실측한 값으로 주장하지 않는다. 활성화 또는 좌표·시각 계약 수정 후 새 비행과 20Hz 렌더 성능을 함께 검증해야 한다.

공식 문서: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_rtx.html

앞선 CPU 비교는 그 입력에 대한 연산량 측정으로 남지만, 원본 GT 오차를 알고리즘 고유 정확도의 우열로 판단하면 안 된다. 공통 센서 시간 불일치가 섞여 있었다.

자료: diagnostic_comparison.csv, correction_summary.json, 각 조건의 raw odometry·state_trace, lidar_map_consistency.json, render_pose_lag.json. 원본 기준 시간 재생과 진단 시각 재생의 LiDAR XYZ 및 원래 도착 스케줄은 별도 검증했다.
''')
print(table)
