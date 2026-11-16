import csv,hashlib,json,subprocess
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parent;WS=ROOT.parents[2]
verification=json.loads((ROOT/'verification.json').read_text());assert verification['all_passed']
runs=verification['runs'];initial=yaml.safe_load((ROOT/'initial.yaml').read_text());final=yaml.safe_load((ROOT/'recommended.yaml').read_text());base=json.loads((ROOT/'baseline-positive/evaluation.json').read_text())
all_results=[json.loads(p.read_text())for p in sorted(ROOT.glob('*/evaluation.json'),key=lambda p:(p.parent/'result.json').stat().st_mtime)]
(ROOT/'summary.json').write_text(json.dumps({'verification':verification,'trials':all_results},indent=2)+'\n')
fig,axes=plt.subplots(3,1,figsize=(12,10),sharex=True)
worst={k:None for k in ('speed_mps','acc_mps2','jerk_mps3')}
for name in ['baseline-positive']+[r['trial']for r in runs]:
 with(ROOT/name/'telemetry/kinematics.csv').open()as f:rows=[r for r in csv.DictReader(f)if r['source']=='truth'and r['state']=='TRACKING']
 t=np.array([float(r['sim_time_s'])for r in rows]);t-=t[0]
 label='Initial 5.5 m/s'if name=='baseline-positive'else name.rsplit('-',1)[-1]+' verified run'
 for ax,key in zip(axes,('speed_mps','acc_mps2','jerk_mps3')):
  values=np.array([float(r[key])for r in rows]);display=values.copy();display[np.r_[False,np.diff(t)>.1]]=np.nan
  ax.plot(t,display,label=label,lw=1,alpha=.8)
  i=int(np.argmax(values))
  if name!='baseline-positive'and(worst[key]is None or values[i]>worst[key][1]):worst[key]=(t[i],values[i])
for ax,key,limit,label in zip(axes,('speed_mps','acc_mps2','jerk_mps3'),('max_vel','max_acc','max_jerk'),('Speed (m/s)','Acceleration (m/s²)','Jerk (m/s³)')):
 x,y=worst[key];ax.plot(x,y,'ro',ms=5);ax.annotate(f'Verified worst MAX {y:.3f}',(x,y),xytext=(.02,.92),textcoords='axes fraction',va='top',color='darkred',bbox={'facecolor':'white','alpha':.85,'edgecolor':'none'},arrowprops={'arrowstyle':'->','color':'darkred'})
 ax.axhline(final['traj_opt']['boundary'][limit],ls='--',color='gray',lw=1,label='Configured boundary (not physical hard limit)');ax.set_ylabel(label);ax.set_ylim(bottom=0);ax.grid(alpha=.25);ax.legend(fontsize=8,loc='upper right')
axes[-1].set_xlabel('Time since TRACKING began (simulation s)');fig.suptitle('GUI high_speed / Super-LIO — physical body, TRACKING\nAcceleration / jerk: ~0.21 s Savitzky–Golay vector derivatives');fig.tight_layout(rect=(0,0,1,.96));fig.savefig(ROOT/'comparison.png',dpi=160);plt.close(fig)
lines=['# high_speed + Super-LIO 패널티 튜닝 — 2026-11-13','',f"최종 채택: **`{verification['selected']}`**, 새 Isaac Sim GUI + RViz2 세션 **3회 연속 검증 통과**. 이 머신의 고정 high_speed 왕복에서 시험한 조합 중 반복 검증을 통과한 가장 빠른 조합이며, 연속 파라미터 공간의 전역 최적값을 증명한 것은 아니다.",'','기준은 이번 요청 시점의 양수 exp jerk/attract 및 backup jerk 설정이다. 이전 음수 패널티 시험을 기준으로 섞지 않았다. 최종 설정에서 OMMPC·Super-LIO 알고리즘·지도·기체 형상·온보드 안전 임계값은 변경하지 않았다. 주로 패널티를 비교하고 속도/가속도/저크 boundary를 단계적으로 올렸다. 보조 비교로 각속도 한계와 MPC 20ms 주기를 각 1회 시험했지만 채택하지 않고 원복했다.','', '## 최종 boundary','', '```yaml','boundary:',*[f'  {k}: {v}'for k,v in final['traj_opt']['boundary'].items()],'```','', '## 패널티 설정','', '일괄 축소가 아니라 시간 비용과 동역학·경로 벌점의 상대 크기를 비교했다. 최종 조합은 여러 항을 함께 바꾼 실험이므로 개별 항의 독립적인 인과 효과를 확정하지 않는다.']
for family in ('exp_traj','backup_traj'):
 lines+=['',f'### {family}','','| 항목 | 처음 값 | 최종 값 |','|---|---:|---:|']
 for k,v in final['traj_opt'][family].items():
  if k.startswith('penna_'):
   before=initial['traj_opt'][family][k];after=f'{v:g}'
   lines.append(f'| {k} | {before:g} | '+('**'+after+'**'if before!=v else after)+' |')
lines+=['','## 새 세션 반복 검증','','| 실행 | 추종 sim s | 벽시계 s | 실제 최고 / p95 m/s | 최고/설정 | 실제 acc / jerk 최대 | LIO 최대 m | 여유 하한 m |','|---|---:|---:|---|---:|---|---:|---:|']
for r in runs:
 motion=r['physical_motion'];lines.append(f"| [{r['trial']}]({r['trial']}/evaluation.json) | {r['tracking_sim_s']:.2f} | {r['tracking_wall_s']:.2f} | {r['actual_peak_mps']:.3f} / {r['actual_p95_mps']:.3f} | {r['peak_speed_attainment']:.3f} | {motion['a']['value']:.3f} m/s² / {motion['j']['value']:.3f} m/s³ | {r['flight_lio_max_m']:.3f} | {r['conservative_body_clearance_m']:.3f} |")
times=[r['tracking_sim_s']for r in runs];baseline_runs=[json.loads((ROOT/f'verify-baseline-positive-{i}/evaluation.json').read_text())for i in range(1,4)];baseline_mean=float(np.mean([r['tracking_sim_s']for r in baseline_runs]));improvement=(1-np.mean(times)/baseline_mean)*100
lines+=['',f"현재 기준 탐색 비행: {base['tracking_sim_s']:.2f} sim s, 실제 최고 {base['actual_peak_mps']:.3f}m/s. 기준 설정의 별도 3회 평균은 {baseline_mean:.2f} sim s. 최종 3회 평균 {np.mean(times):.2f} sim s, 범위 {min(times):.2f}–{max(times):.2f}s. 평균 기준 시간 단축 {improvement:.1f}%. 이륙·착륙 시간은 제외하고 회항·계획 대기를 포함한 TRACKING 구간이다.",'','최고 속도만 짧게 찍은 결과와 구분하기 위해 실제 속도의 95백분위, 설정 대비 달성률, 설정 90% 이상 비행한 표본 비율도 각 evaluation.json에 저장했다. 최종 후보의 매 반복에서 peak/설정 ≥0.9, p95/설정 ≥0.85, peak/설정 ≤1.15를 요구했다. 설정 15m/s에 실제 5m/s인 조합은 이 기준으로 채택할 수 없다.','', 'acc/jerk 값은 물리 기체의 body 원점 속도 벡터를 약 0.21초 평활 미분한 실측치다. nominal boundary와 같지 않으며 표에 실제 초과량을 숨기지 않았다. reference 궤적의 analytic 최대값은 각 metrics.json에 별도로 저장했다.','', '![주행 비교](comparison.png)','', '## 판정과 재현','','- Isaac Sim GUI + RViz2를 모두 실행하고 매 trial마다 새 simulator를 시작했다. 중간 GUI 화면·window tree를 보관했다.','- physical ground truth는 진단·시험 중단 판단에만 사용하고 계획·제어에는 주입하지 않았다. controller mode는 rate-only인지 독립 확인했다.','- 중도 착륙/FAULT/STOPPED, 미완주, 물리 목표 미도달은 순위에서 제외했다. 두 목표의 실제 위치 오차 0.8m 이하, 최종 정상 착륙·disarm을 요구했다.','- 사용자가 정상 완주·충돌 여유를 우선하고 위치 오차를 함께 보고하도록 선택했다. 따라서 기존 LIO 최대 오차 0.5m/RMS 0.25m는 최종 탈락 기준으로 사용하지 않았다. odom 최대 stamp 공백 50ms 이하·수신 공백 200ms 이하와 기존 온보드 보호는 유지했다.','- 실제 body 중심 선분을 1cm 이하 간격으로 검사한 PCD 거리에서 body 반경 0.33m·복셀 재구성 상한 0.086603m·선분 반간격 0.005m를 빼고 여유 하한 0.1m 이상을 요구했다. 장애물 KDTree에서는 지면을 제외하고, 주행 중 body 최소 높이에서 0.33m를 뺀 지면 여유도 별도로 0.1m 이상인지 검사했다. 음수는 보수적 무접촉 입증 실패이며 PhysX 접촉을 직접 센 결과가 아니다.','- 단발 통과 후보를 빠른 순서대로 다시 3회 검증했다. 사용자의 기준 변경 전에 위치 오차 2m로 진단 중단한 시험은 새 기준에서 미완료 탐색으로 구분한다. 이후에는 위치 오차만으로 시험을 종료하지 않고 실제 시나리오 이탈·기존 온보드 보호·미완주·물리 목표 미도달·충돌 여유 부족을 확인했다. 모든 실패와 중단 기록을 보존했다.','', '```bash','./run_super_sim.sh high_speed','# 다른 터미널','./run_super.sh --scenario high_speed --backend super_lio --execute','# 종료 후 실제 움직임 그래프','./show_super_flight.sh','```','', '적용 파일은 `src/zav_super/config/high_speed_super_lio.yaml` 및 동일한 install 복사본이다. 기존 명령은 그대로이며 dense·다른 LIO 프로파일은 바꾸지 않았다.','', '## 전체 탐색 기록','','실패한 비행의 부분 시간·순간 최고속도는 완주 성능으로 제시하지 않는다. 진단용 상세 숫자는 각 evaluation.json에서 확인할 수 있다.','','| 시험 | v / a / j | 완주·여유 통과 | 순위 대상 시간 s | 실제 최고 m/s | 이유 |','|---|---|---|---:|---:|---|']
for r in all_results:
 eligible=r['eligible'];bound=r['boundary'];time=f"{r['ranking_time_sim_s']:.2f}"if eligible else'—';speed=f"{r.get('actual_peak_mps',0):.3f}"if r['healthy_complete']else'—'
 reason='통과'if eligible else '; '.join(r.get('faults',[]))
 if not reason:
  if r.get('termination')=='physical/LIO divergence >2m':reason='기준 변경 전 위치오차 진단 중단'
  elif not r.get('completed')or not r.get('landed'):reason='미완주/중도 시험 종료'
  elif r.get('conservative_body_clearance_m',-999)<.1 or r.get('floor_clearance_bound_m',-999)<.1:reason='물리 충돌 여유 기준 미달'
  elif max(r.get('physical_goal_min_distance_m',[999]))>.8:reason='실제 목표 위치 미도달'
  elif not r['speed_matches_setting']:reason='실제 속도와 설정 불일치'
  else:reason='GUI/수신/완주 연속성 검사 미달'
 lines.append(f"| [{r['trial']}]({r['trial']}/evaluation.json) | {bound['max_vel']}/{bound['max_acc']}/{bound['max_jerk']} | {r['healthy_complete']} | {time} | {speed} | {reason} |")
lines+=['','## 원본 기록','','[전체 결과](summary.json) · [완주 우선 반복 검증·선정](verification.json) · [이전 정확도 기준 검증](verification-previous-strict.json) · [사용자 선택 기준](selection-policy.json) · [시작 파일 해시](initial-hashes.json) · [권장 설정 전체](recommended.yaml). 각 trial의 telemetry 폴더에 report.html, motion.png, kinematics.csv, samples.jsonl 및 실제 사용한 planner metadata가 있다. flight.jsonl/odom-events.jsonl은 독립적인 위치·수신 품질 진단이다. 로그는 로컬 *.log로 남아 있다.']
(ROOT/'README.md').write_text('\n'.join(lines)+'\n')
initial_hashes=json.loads((ROOT/'initial-hashes.json').read_text());target='src/zav_super/config/high_speed_super_lio.yaml'
checks={'three_fresh_runs_passed':all(r['eligible']and r['profile_matches_seed']for r in runs),'user_accuracy_preference_recorded':json.loads((ROOT/'selection-policy.json').read_text())['accuracy_is_reported_not_a_gate'],'floor_clearance_passed':all(r['floor_clearance_bound_m']>=.1 for r in runs),'no_intermediate_landing':all(r['uninterrupted_mission']for r in runs),'gui_both_verified':all(r['gui_verified']for r in runs),'speed_matches_setting':all(r['speed_matches_setting']for r in runs),'source_matches_recommendation':yaml.safe_load((WS/target).read_text())==final,'installed_matches_source':(WS/target).read_bytes()==(WS/'install/zav_super/share/zav_super/config/high_speed_super_lio.yaml').read_bytes(),'other_source_inputs_unchanged':all(hashlib.sha256((WS/p).read_bytes()).hexdigest()==h for p,h in initial_hashes.items()if p!=target)}
(ROOT/'acceptance.json').write_text(json.dumps(checks,indent=2)+'\n');assert all(checks.values()),checks
print('Report and acceptance PASS:',ROOT/'README.md')
