#!/usr/bin/env python3
"""Prepare identical raw-point subsets; never reads ground truth."""
from pathlib import Path
import sqlite3,struct,json,hashlib
import numpy as np
import yaml
R=Path(__file__).resolve().parents[2];P=R/'.cache/lio-fixed-20261102';O=R/'reports/lio-fixed-20261102';O.mkdir(exist_ok=True)
source=sqlite3.connect(next((P/'dataset').glob('*.db3')))
target=P/'common.db3'
assert not target.exists()
c=sqlite3.connect(target);source.backup(c)
rows=[]
for mid,raw in c.execute('select id,data from messages where topic_id=(select id from topics where name like "%/lidar")').fetchall():
 n,=struct.unpack_from('<I',raw,12);pos=4+((12+n+7)//8)*8
 count,=struct.unpack_from('<I',raw,pos+16)
 dtype=np.dtype({'names':['dt','x','y','z','intensity','tag','line'], 'formats':['<u4','<f4','<f4','<f4','u1','u1','u1'],'offsets':[0,4,8,12,16,17,18],'itemsize':20})
 # Fast DDS omits the final record's trailing CDR padding byte.
 raw=raw+b'\0'*max(0,pos+20+count*20-len(raw))
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
for alg in []:
 cfg=yaml.safe_load((P/f'{alg}-matched.yaml').read_text());v=next(iter(cfg.values()))['ros__parameters']
 if alg=='fast_livo':v['preprocess']['point_filter_num']=1
 else:v['lio.sensor.filter_rate']=1
 (P/f'{alg}-common.yaml').write_text(yaml.safe_dump(cfg))
