#!/usr/bin/env python3
"""Accuracy of recorded-input diagnostic interventions. Not a speed benchmark."""
from pathlib import Path
import json,math
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[2];P=R/'.cache/lio-comparison-20261102';O=R/'reports/lio-spike-diagnosis-20261102'
a=[json.loads(l) for f in sorted(P.glob('truth-*.jsonl')) for l in f.read_text().splitlines()];gt_t=np.array([x['t'] for x in a]);gt_p=np.array([x['p'] for x in a]);gt_slerp=Slerp(gt_t,Rotation.from_quat([x['q'] for x in a]));grid=np.arange(51.1,80.75,.1);GTp=np.column_stack([np.interp(grid,gt_t,gt_p[:,i]) for i in range(3)]);GTr=gt_slerp(grid)
# Reuse the exact alignment/interpolation implementation of the comparison.
s=(R/'benchmarks/lio_compare/analyze_common.py').read_text();f=s[s.index('def trajectory('):s.index('source_cache=')];exec(f)
series={};rows=[];prepared={}
for group in ['raw','frame-intervention','no-deskew','oracle-deskew','boundary-fix','deskew-shift','imu-time-shift','snapshot-time']:
 for alg in ['super_lio','fast_livo']:
  folder=O/group/f'common-{alg}-1'
  if not(folder/'run.json').exists():continue
  source=[json.loads(l) for l in (folder/'odometry.jsonl').read_text().splitlines()]
  for kind in ['imu','lidar']:prepared[(group,alg,kind)]=(source,trajectory(source,kind)[3])
for kind in ['imu','lidar']:
 mask=np.logical_and.reduce([m for (group,alg,k),(source,m) in prepared.items() if k==kind])
 for (group,alg,k),(source,_) in prepared.items():
  if k!=kind:continue
  tt,pp,rr,valid,n=trajectory(source,kind,mask);ep=np.linalg.norm(pp-GTp[valid],axis=1);er=np.rad2deg((GTr[valid].inv()*rr).magnitude())
  d={}
  for x in source:
   if x['kind']==kind:d.setdefault(x['t'],x)
  ordered=sorted(d.values(),key=lambda x:x['t']);t=np.array([x['t'] for x in ordered]);p=np.array([x['p'] for x in ordered]);dt=np.diff(t);delta=np.linalg.norm(np.diff(p,axis=0),axis=1);m=(t[1:]>51.01)&(t[1:]<80.75)&(dt<.0075)
  rows.append(dict(group=group,algorithm=alg,kind=kind,rmse_cm=100*np.sqrt(np.mean(ep**2)),p95_cm=100*np.quantile(ep,.95),max_cm=100*ep.max(),rotation_rmse_deg=np.sqrt(np.mean(er**2)),max_5ms_position_change_cm=float(100*delta[m].max()) if m.any() else None,coverage_percent=valid.mean()*100))
  series[(group,alg,kind)]=(tt,ep,er)
result=pd.DataFrame(rows);result.to_csv(O/'diagnostic_comparison.csv',index=False);print(result.to_string(index=False))
fig,axes=plt.subplots(2,1,figsize=(12,7),constrained_layout=True)
for ax,alg in zip(axes,['super_lio','fast_livo']):
 for group in ['raw','frame-intervention','no-deskew','oracle-deskew','boundary-fix','deskew-shift','imu-time-shift','snapshot-time']:
  if (group,alg,'imu') not in series:continue
  tt,ep,er=series[(group,alg,'imu')];ax.plot(tt,ep*100,label=group)
 ax.set(title=alg,xlabel='Simulation time [s]',ylabel='Position error [cm]',xlim=(65.5,75));ax.grid(alpha=.25);ax.legend()
fig.suptitle('Recorded-input diagnosis; GT-assisted interventions are NOT deployment accuracy results');fig.savefig(O/'spike_comparison.png',dpi=150);plt.close(fig)
