#!/usr/bin/env python3
from pathlib import Path
import json
import numpy as np
import pandas as pd
R=Path(__file__).resolve().parents[2]/'reports/lio-spike-diagnosis-20261102'
cols=['stage','obs_t','forward_t']+[f'{s}{i}' for s in ['p','v','ba','bg','g','fp','fv'] for i in range(3)]+[f'{s}{i}' for s in ['q','fq'] for i in range(4)]
results=[]
for group in ['raw','frame-intervention','no-deskew','oracle-deskew','boundary-fix','snapshot-time']:
 path=R/group/'common-super_lio-1/state_trace.csv'
 if not path.exists():continue
 d=pd.read_csv(path,header=None,names=cols);pairs=[]
 for t,a in d[d.stage!='forward'].groupby('obs_t',sort=False):
  if not 51<t<80.75:continue
  if len(a)<3:continue
  before=a[a.stage=='observe_before'].iloc[0];corrected=a[a.stage=='corrected_before_reprop'].iloc[0];after=a[a.stage=='observe_after'].iloc[0]
  dist=lambda x,y,prefix:float(np.linalg.norm(x[[f'{prefix}{i}' for i in range(3)]].to_numpy(dtype=float)-y[[f'{prefix}{i}' for i in range(3)]].to_numpy(dtype=float)))
  pairs.append(dict(t=t,lidar_position_correction_cm=100*dist(corrected,before,'p'),velocity_correction_mps=dist(corrected,before,'v'),forward_rebase_cm=100*dist(after,before,'fp'),forward_time_before=before.forward_t,forward_time_after=after.forward_t,bias_acc_change=dist(corrected,before,'ba'),bias_gyro_change=dist(corrected,before,'bg')))
 z=pd.DataFrame(pairs);z.to_csv(R/f'{group}-corrections.csv',index=False);top=z.nlargest(8,'forward_rebase_cm');print(group,top.to_string(index=False));results.append(dict(group=group,max_forward_rebase_cm=float(z.forward_rebase_cm.max()),max_lidar_correction_cm=float(z.lidar_position_correction_cm.max())))
(R/'correction_summary.json').write_text(json.dumps(results,indent=2))
