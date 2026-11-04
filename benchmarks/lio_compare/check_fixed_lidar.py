#!/usr/bin/env python3
"""Point-to-plane consistency against a stationary reference cloud, using PhysX poses."""
from pathlib import Path
import json,sqlite3,struct
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from scipy.spatial import cKDTree
R=Path(__file__).resolve().parents[2];P=R/'.cache/lio-fixed-20261102';O=R/'reports/lio-fixed-20261102'
flight=[json.loads(l) for l in (P/'capture.jsonl').read_text().splitlines()];starts={stage:min(x['truth']['sim'] for x in flight if x['stage']==stage) for stage in ['return','yaw_90']};windows=[(starts['return'],starts['return']+1.8),(starts['yaw_90'],starts['yaw_90']+1.35)]
a=[json.loads(l) for f in sorted(P.glob('truth-*.jsonl')) for l in f.read_text().splitlines()];gt=np.array([x['t'] for x in a]);gp=np.array([x['p'] for x in a]);sl=Slerp(gt,Rotation.from_quat([x['q'] for x in a]));extr=np.array([-.011,-.02329,-.04412]);c=sqlite3.connect(P/'common.db3');cloud=[];tm=[]
for raw, in c.execute('select data from messages where topic_id=(select id from topics where name like "%/lidar")'):
 sec,ns=struct.unpack_from('<iI',raw,4);start=sec+ns*1e-9
 if not(any(lo<start<hi for lo,hi in [(40,50)]+windows)):continue
 n,=struct.unpack_from('<I',raw,12);pos=4+((12+n+7)//8)*8;count,=struct.unpack_from('<I',raw,pos+16)
 a=np.frombuffer(raw,dtype=np.dtype({'names':['dt','x','y','z'],'formats':['<u4','<f4','<f4','<f4'],'offsets':[0,4,8,12],'itemsize':20}),count=count,offset=pos+20);cloud.append(np.column_stack([a[x] for x in 'xyz'])[::3]);tm.append((start+a['dt']*1e-9)[::3])
xyz=np.concatenate(cloud);t=np.concatenate(tm)
render_t=np.arange(.01,gt.max()-.05,.05);render_t=render_t[render_t>=gt.min()];render_rot=Slerp(render_t,sl(render_t));render_p=np.column_stack([np.interp(render_t,gt,gp[:,i]) for i in range(3)])
def render_world(times,pts):return render_rot(times).apply(pts+extr)+np.column_stack([np.interp(times,render_t,render_p[:,i]) for i in range(3)])
def world(times,pts):return sl(times).apply(pts+extr)+np.column_stack([np.interp(times,gt,gp[:,i]) for i in range(3)])
static=t<50;ref=world(t[static],xyz[static]);_,idx=np.unique(np.floor(ref/.05).astype(np.int32),axis=0,return_index=True);ref=ref[idx];tree=cKDTree(ref);dist,idx=tree.query(ref,k=12,workers=2);neighbors=ref[idx];center=neighbors.mean(axis=1);cov=np.einsum('nki,nkj->nij',neighbors-center[:,None,:],neighbors-center[:,None,:])/12;ev,vec=np.linalg.eigh(cov);normal=vec[:,:,0];quality=(ev[:,0]<.02*ev[:,1])&(dist[:,-1]<.4)
rows=[]
for lo,hi in windows:
 m=(t>=lo)&(t<hi);tt=t[m];pts=xyz[m];nominal=world(tt,pts);dd,ii=tree.query(nominal,workers=2);keep=(dd<.4)&quality[ii];tt=tt[keep];pts=pts[keep]
 for mode in ['per_point','render_interp','frame_start','frame_end']:
  for lag in (np.arange(-.05,.0501,.005) if mode in ['per_point','render_interp'] else [0]):
   times=tt+lag if mode in ['per_point','render_interp'] else .01+np.ceil((tt-.01)/.05)*.05+(0 if mode=='frame_end' else -.05)
   w=render_world(times,pts) if mode=='render_interp' else world(times,pts);dd,ii=tree.query(w,workers=2);e=np.abs(np.einsum('ni,ni->n',w-center[ii],normal[ii]));e=np.minimum(e,.5)
   rows.append(dict(window=[lo,hi],mode=mode,lag_s=float(lag),n=len(e),point_plane_rmse_cm=float(100*np.sqrt(np.mean(e**2))),median_cm=float(100*np.median(e)),p95_cm=float(100*np.quantile(e,.95))))
(O/'lidar_map_consistency.json').write_text(json.dumps(rows,indent=2))
for win in [list(x) for x in windows]:
 z=[x for x in rows if x['window']==win];print(win,'best',sorted(z,key=lambda x:x['point_plane_rmse_cm'])[:4]);print('nominal',[x for x in z if x['mode']!='per_point' or abs(x['lag_s'])<1e-6])
