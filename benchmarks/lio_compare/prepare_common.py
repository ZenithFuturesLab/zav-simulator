#!/usr/bin/env python3
"""Prepare identical raw-point subsets; never reads ground truth."""
from pathlib import Path
import sqlite3,struct,json,hashlib
import numpy as np
import yaml
R=Path(__file__).resolve().parents[2];P=R/'.cache/lio-comparison-20261102';O=R/'reports/lio-common-20261102';O.mkdir(exist_ok=True)
source=sqlite3.connect(next((P/'dataset').glob('*.db3')))
target=P/'common.db3'
assert not target.exists()
c=sqlite3.connect(target);source.backup(c)
rows=[]
for mid,raw in c.execute('select id,data from messages where topic_id=(select id from topics where name like "%/lidar")').fetchall():
 n,=struct.unpack_from('<I',raw,12);pos=4+((12+n+7)//8)*8
 count,=struct.unpack_from('<I',raw,pos+16)
 dtype=np.dtype({'names':['dt','x','y','z','intensity','tag','line'], 'formats':['<u4','<f4','<f4','<f4','u1','u1','u1'],'offsets':[0,4,8,12,16,17,18],'itemsize':20})
 a=np.frombuffer(raw,dtype=dtype,count=count,offset=pos+20)
 xyz=np.column_stack([a[x] for x in ['x','y','z']]);d2=(xyz**2).sum(axis=1)
 valid=np.flatnonzero(np.isfinite(xyz).all(axis=1)&(d2>.25)&(d2<10000)&(a['line']<4)&np.isin(a['tag']&0x30,[0,16]))
 # One real return per voxel, first in acquisition order. Retain scan endpoints to preserve timing.
 vox=np.floor(xyz[valid]/.5).astype(np.int64);_,indices=np.unique(vox,axis=0,return_index=True)
 keep=np.unique(np.r_[valid[indices],valid[0],valid[-1]])
 subset=a[keep].tobytes();out=bytearray(raw[:pos+20]);struct.pack_into('<I',out,pos+8,len(keep));struct.pack_into('<I',out,pos+16,len(keep));out.extend(subset)
 c.execute('update messages set data=? where id=?',(bytes(out),mid))
 rows.append(dict(id=mid,original=count,selected=len(keep),sha256=hashlib.sha256(subset).hexdigest()))
c.commit();c.close();source.close()
(O/'common_point_manifest.json').write_text(json.dumps(rows,indent=2))
for alg in ['fast_livo','super_lio']:
 cfg=yaml.safe_load((P/f'{alg}-matched.yaml').read_text());v=next(iter(cfg.values()))['ros__parameters']
 if alg=='fast_livo':v['preprocess']['point_filter_num']=1
 else:v['lio.sensor.filter_rate']=1
 (P/f'{alg}-common.yaml').write_text(yaml.safe_dump(cfg))
# Benchmark-only switch: keep exactly the shared selected returns through each native deskewer.
f=P/'bench_src/fast_livo/src/LIVMapper.cpp';s=f.read_text();s=s.replace('downSizeFilterSurf.filter(*feats_down_body);','if (std::getenv("ZAV_COMMON_POINTS")) *feats_down_body = *feats_undistort;\n  else downSizeFilterSurf.filter(*feats_down_body);');f.write_text(s)
f=P/'bench_src/super_lio/src/lio/super_lio.cpp';s=f.read_text().replace('voxel_grid_fliter_.filter(ds_undistort_);','if (std::getenv("ZAV_COMMON_POINTS")) *ds_undistort_ = *scan_undistort_full_;\n  else voxel_grid_fliter_.filter(ds_undistort_);');f.write_text(s)
# FAST's regular sparse-time heuristic assumes dense raw returns. Common input retains true timestamps.
f=P/'bench_src/fast_livo/src/preprocess.cpp';s=f.read_text().replace('if (i == 0)\n          pl_full[i].curvature','if (std::getenv("ZAV_COMMON_POINTS")) {}\n        else if (i == 0)\n          pl_full[i].curvature');f.write_text(s)
s=(R/'benchmarks/lio_compare/replay.py').read_text().replace("choices=['deployed','matched']","choices=['common']").replace("ROOT/'reports/lio-comparison-20261102/raw'","ROOT/'reports/lio-common-20261102/raw'").replace("next((P/'dataset').glob('*.db3'))","P/'common.db3'").replace("events.sort(key=lambda x:(x[0],x[1]));first=", "events=[x for x in events if x[0]*1e-9<=80.75]\nevents.sort(key=lambda x:(x[0],x[1]));first=").replace("env=dict(os.environ);", "env=dict(os.environ);env['ZAV_COMMON_POINTS']='1';")
# Counts describe actually replayed prefix.
s=s.replace("# Validate the direct CDR", "counts={'imu':sum(x[1]==0 for x in events),'lidar':sum(x[1]==1 for x in events)}\n# Validate the direct CDR")
(R/'benchmarks/lio_compare/replay_common.py').write_text(s)
print('Common subsets:',len(rows),'mean points',np.mean([x['selected'] for x in rows]))
