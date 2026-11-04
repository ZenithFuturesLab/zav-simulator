#!/usr/bin/env python3
"""GT-assisted scan deskew ablation; diagnostic only, never deploy as an estimator."""
from pathlib import Path
import json,sqlite3,struct
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
R=Path(__file__).resolve().parents[2];P=R/'.cache/lio-comparison-20261102'
a=[json.loads(l) for f in sorted(P.glob('truth-*.jsonl')) for l in f.read_text().splitlines()];gt=np.array([x['t'] for x in a]);gp=np.array([x['p'] for x in a]);sl=Slerp(gt,Rotation.from_quat([x['q'] for x in a]));extr=np.array([-.011,-.02329,-.04412])
s=sqlite3.connect(P/'common.db3');out=P/'oracle-deskew.db3';assert not out.exists();c=sqlite3.connect(out);s.backup(c)
for mid,raw in c.execute('select id,data from messages where topic_id=(select id from topics where name like "%/lidar")').fetchall():
 sec,ns=struct.unpack_from('<iI',raw,4);start=sec+ns*1e-9
 if start<gt.min() or start>81:continue
 n,=struct.unpack_from('<I',raw,12);pos=4+((12+n+7)//8)*8;count,=struct.unpack_from('<I',raw,pos+16);buf=bytearray(raw)
 a=np.frombuffer(buf,dtype=np.dtype({'names':['dt','x','y','z'],'formats':['<u4','<f4','<f4','<f4'],'offsets':[0,4,8,12],'itemsize':20}),count=count,offset=pos+20);xyz=np.column_stack([a[x] for x in 'xyz']);t=start+a['dt']*1e-9;end=t.max();world=sl(t).apply(xyz+extr)+np.column_stack([np.interp(t,gt,gp[:,i]) for i in range(3)]);p=np.array([np.interp(end,gt,gp[:,i]) for i in range(3)]);corrected=sl(end).inv().apply(world-p)-extr
 for i,x in enumerate('xyz'):a[x]=corrected[:,i]
 c.execute('update messages set data=? where id=?',(bytes(buf),mid))
c.commit();c.close();s.close()
s=(R/'benchmarks/lio_compare/replay_no_deskew.py').read_text().replace("P/'common.db3'","P/'oracle-deskew.db3'").replace("ROOT/'reports/lio-spike-diagnosis-20261102/no-deskew'","ROOT/'reports/lio-spike-diagnosis-20261102/oracle-deskew'")
(R/'benchmarks/lio_compare/replay_oracle_deskew.py').write_text(s)
print('Oracle-deskew diagnostic input prepared')
