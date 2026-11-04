#!/usr/bin/env python3
from pathlib import Path
import json
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation,Slerp
R=Path(__file__).resolve().parents[2];P=R/'.cache/lio-fixed-20261102';O=R/'reports/lio-fixed-20261102'
a=[json.loads(l) for p in sorted(P.glob('truth-*.jsonl')) for l in p.read_text().splitlines()];gt=np.array([x['t'] for x in a]);gp=np.array([x['p'] for x in a]);gr=Slerp(gt,Rotation.from_quat([x['q'] for x in a]));f=[json.loads(l) for l in (P/'capture.jsonl').read_text().splitlines()];lo=min(x['truth']['sim'] for x in f if x['stage']=='north_1m');hi=min(x['truth']['sim'] for x in f if x['stage']=='hover_final');rows=[];top=[]
for folder in sorted((O/'raw').glob('*')):
 if not(folder/'run.json').exists():continue
 d={}
 for l in (folder/'odometry.jsonl').read_text().splitlines():
  x=json.loads(l)
  if x['kind']=='imu':d.setdefault(x['t'],x)
 a=sorted(d.values(),key=lambda x:x['t']);t=np.array([x['t'] for x in a]);p=np.array([x['p'] for x in a]);q=Rotation.from_quat([x['q'] for x in a]);m=(t>=5)&(t<=15);rot=gr(t[m]).mean()*q[m].mean().inv();trans=np.column_stack([np.interp(t[m],gt,gp[:,i]) for i in range(3)]).mean(axis=0)-rot.apply(p[m]).mean(axis=0);p=rot.apply(p)+trans;truth=np.column_stack([np.interp(t,gt,gp[:,i]) for i in range(3)]);error=p-truth;dt=np.diff(t);jump=np.linalg.norm(np.diff(error,axis=0),axis=1);mask=(t[1:]>=lo)&(t[1:]<=hi)&(dt<=.03+1e-7);indices=np.flatnonzero(mask);i=indices[np.argmax(jump[mask])]
 row=dict(run=folder.name,max_error_change_cm=float(100*jump[i]),at_sim_s=float(t[i+1]),interval_ms=float(dt[i]*1000),max_raw_ate_cm=float(100*np.linalg.norm(error[(t>=lo)&(t<=hi)],axis=1).max()),gaps_over_7_5_ms=int(((dt>.0075)&(t[1:]>=lo)&(t[1:]<=hi)).sum()),max_publish_gap_ms=float(dt[(t[1:]>=lo)&(t[1:]<=hi)].max()*1000),error_changes_over_5cm=int((jump[mask]>.05).sum()));rows.append(row)
 for i in indices[np.argsort(jump[mask])[-10:]]:top.append(dict(run=folder.name,t=float(t[i+1]),error_change_cm=float(jump[i]*100),dt_ms=float(dt[i]*1000)))
pd.DataFrame(rows).to_csv(O/'high_rate_diagnostics.csv',index=False);pd.DataFrame(top).to_csv(O/'largest_output_changes.csv',index=False);print(pd.DataFrame(rows).to_string(index=False))
