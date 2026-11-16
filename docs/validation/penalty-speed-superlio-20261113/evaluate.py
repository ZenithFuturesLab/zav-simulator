"""Require physical completion/clearance/LIO health before comparing flight time."""
import json,subprocess,sys,csv,re
from pathlib import Path
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parent
for name in sys.argv[1:]:
 folder=ROOT/name
 with (folder/'analysis.log').open('w')as f:r=subprocess.run([sys.executable,str(ROOT/'analyze.py'),str(folder)],stdout=f,stderr=subprocess.STDOUT)
 if r.returncode or not(folder/'metrics.json').exists():
  result={'trial':name,'accepted':False,'analysis_error':(folder/'analysis.log').read_text()[-1800:]}
 else:result=json.loads((folder/'metrics.json').read_text())
 config=yaml.safe_load((folder/'loaded-high_speed.yaml').read_text());bound=config['traj_opt']['boundary'];result['boundary']=bound
 if(folder/'loaded-ommpc.yaml').exists():result['controller']=yaml.safe_load((folder/'loaded-ommpc.yaml').read_text())['zav_ommpc_px4']['ros__parameters']
 telemetry=folder/'telemetry'
 if (telemetry/'samples.jsonl').exists():
  # Recompute with one common derivative filter, after owned processes have exited.
  with (folder/'telemetry-analysis.log').open('w')as f:subprocess.run([sys.executable,str(ROOT.parents[2]/'simulator/tools/super/flight_metrics.py'),str(telemetry)],stdout=f,stderr=subprocess.STDOUT)
 if (telemetry/'summary.json').exists():
  motion=json.loads((telemetry/'summary.json').read_text());result['physical_motion']=motion['sources'].get('truth',{}).get('tracking',{})
  rows=[]
  with (telemetry/'kinematics.csv').open()as f:
   rows=[r for r in csv.DictReader(f)if r['source']=='truth'and r['state']=='TRACKING']
  if rows:
   v=np.array([float(r['speed_mps'])for r in rows]);result.update(actual_peak_mps=float(v.max()),actual_p95_mps=float(np.percentile(v,95)),peak_speed_attainment=float(v.max()/bound['max_vel']),p95_speed_attainment=float(np.percentile(v,95)/bound['max_vel']),time_fraction_above_90pct_cap=float(np.mean(v>=.9*bound['max_vel'])))
 windows=(folder/'windows.txt').read_text()if(folder/'windows.txt').exists()else'';log=(folder/'launch.log').read_text(errors='replace')
 result['gui_verified']='Isaac Sim'in windows and'RViz'in windows and 'process has died'not in log
 result['controller_feedback_steps_ms']=[float(x)for x in re.findall(r'OMMPC feedback step=([0-9.]+) ms',log)]
 result['controller_solver_ms']=[float(x)for x in re.findall(r'solver=([0-9.]+) ms',log)]
 result['optimizer_failure_lines']=sum('OPT_FAILED'in line or'TrajOpt failed'in line for line in log.splitlines())
 phases=[]
 for line in (folder/'flight.jsonl').read_text().splitlines():
  state=json.loads(line).get('controller','UNKNOWN')
  if not phases or phases[-1]!=state:phases.append(state)
 result['controller_phases']=phases;landing_indices=[i for i,s in enumerate(phases)if s=='LANDING']
 result['uninterrupted_mission']=len(landing_indices)==1 and 'TRACKING'not in phases[landing_indices[0]+1:] and bool(result.get('completed'))
 result['healthy_complete']=bool(result.get('accepted')and result['gui_verified']and result['uninterrupted_mission'])
 result['speed_matches_setting']=result.get('peak_speed_attainment',0)>=.9 and result.get('p95_speed_attainment',0)>=.85 and result.get('peak_speed_attainment',99)<=1.15
 result['eligible']=result['healthy_complete']and result['speed_matches_setting']
 result['ranking_time_sim_s']=result.get('tracking_sim_s')if result['eligible']else None
 from criteria import apply_completion_policy
 result=apply_completion_policy(result)
 (folder/'evaluation.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({k:result.get(k)for k in ('trial','healthy_complete','eligible','ranking_time_sim_s','actual_peak_mps','actual_p95_mps','peak_speed_attainment','flight_lio_max_m','conservative_body_clearance_m','faults')}),flush=True)
