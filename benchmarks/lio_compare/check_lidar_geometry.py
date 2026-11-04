#!/usr/bin/env python3
"""Static-floor consistency under different point-pose timing assumptions."""
from pathlib import Path
import json,sqlite3,struct
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
R=Path(__file__).resolve().parents[2];P=R/'.cache/lio-comparison-20261102';O=R/'reports/lio-spike-diagnosis-20261102'
a=[json.loads(l) for f in sorted(P.glob('truth-*.jsonl')) for l in f.read_text().splitlines()];gt=np.array([x['t'] for x in a]);gp=np.array([x['p'] for x in a]);sl=Slerp(gt,Rotation.from_quat([x['q'] for x in a]));extr=np.array([-.011,-.02329,-.04412])
c=sqlite3.connect(P/'common.db3');cloud=[];tm=[]
for raw, in c.execute('select data from messages where topic_id=(select id from topics where name like "%/lidar")'):
 sec,ns=struct.unpack_from('<iI',raw,4);t=sec+ns*1e-9
 if not(40<t<50 or 51<t<52 or 58<t<60 or 65.7<t<67.5 or 73.25<t<74.6):continue
 n,=struct.unpack_from('<I',raw,12);pos=4+((12+n+7)//8)*8;count,=struct.unpack_from('<I',raw,pos+16)
 a=np.frombuffer(raw,dtype=np.dtype({'names':['dt','x','y','z'],'formats':['<u4','<f4','<f4','<f4'],'offsets':[0,4,8,12],'itemsize':20}),count=count,offset=pos+20)
 cloud.append(np.column_stack([a[x] for x in 'xyz']));tm.append(t+a['dt']*1e-9)
xyz=np.concatenate(cloud);t=np.concatenate(tm)
def world(times):
 return sl(times).apply(xyz+extr)+np.column_stack([np.interp(times,gt,gp[:,i]) for i in range(3)])
w=world(t);static=(t>40)&(t<50);hist,bins=np.histogram(w[static,2],bins=np.arange(-.5,1.01,.01));z0=float((bins[hist.argmax()]+bins[hist.argmax()+1])/2);print('dominant static floor z',z0)
mask_floor=np.abs(w[:,2]-z0)<.5
# Compare the same candidate floor returns for every timing hypothesis.
rows=[]
for mode in ['per_point','frame_end','frame_start']:
 for lag in (np.arange(-.10,.1001,.01) if mode=='per_point' else [0]):
  tt=t+lag if mode=='per_point' else (.01+np.ceil((t-.01)/.05)*.05 + (0 if mode=='frame_end' else -.05))
  ww=world(tt)
  for lo,hi in [(40,50),(51,52),(58,60),(65.7,67.5),(73.25,74.6)]:
   m=mask_floor&(t>=lo)&(t<hi);e=ww[m,2]-z0;rows.append(dict(mode=mode,lag_s=float(lag),window=[lo,hi],n=int(m.sum()),floor_rmse_cm=float(100*np.sqrt(np.mean(e**2))),floor_p95_cm=float(100*np.quantile(abs(e),.95))))
(O/'lidar_floor_consistency.json').write_text(json.dumps(rows,indent=2))
for win in [[40,50],[65.7,67.5],[73.25,74.6]]:
 z=[x for x in rows if x['window']==win];print(win,'best',sorted(z,key=lambda x:x['floor_rmse_cm'])[:3]);print('nominal',[x for x in z if x['mode']!='per_point' or abs(x['lag_s'])<1e-6])
