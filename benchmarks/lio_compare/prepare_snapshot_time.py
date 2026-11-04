#!/usr/bin/env python3
"""Diagnostic: associate returns with observed render-frame start geometry.
Does not read GT. Preserves raw point IDs and original packet availability.
"""
from pathlib import Path
import sqlite3,struct,json
import numpy as np
R=Path(__file__).resolve().parents[2];P=R/'.cache/lio-comparison-20261102';s=sqlite3.connect(P/'common.db3');p=P/'snapshot-time.db3';assert not p.exists();c=sqlite3.connect(p);s.backup(c);availability={}
for mid,raw in c.execute('select id,data from messages where topic_id=(select id from topics where name like "%/lidar")').fetchall():
 sec,ns=struct.unpack_from('<iI',raw,4);start=sec*10**9+ns;n,=struct.unpack_from('<I',raw,12);pos=4+((12+n+7)//8)*8;count,=struct.unpack_from('<I',raw,pos+16);b=bytearray(raw);dt=np.ndarray((count,),dtype='<u4',buffer=b,offset=pos+20,strides=(20,));times=start+dt.astype(np.int64);availability[mid]=int(times.max())
 snapshot=10_000_000+((times-10_000_000-1)//50_000_000)*50_000_000
 origin=int(snapshot.min());struct.pack_into('<iI',b,4,*divmod(origin,10**9));struct.pack_into('<Q',b,pos,origin);dt[:]=(snapshot-origin).astype('<u4');c.execute('update messages set data=? where id=?',(bytes(b),mid))
c.commit();c.close();s.close();(P/'snapshot-availability.json').write_text(json.dumps(availability))
s=(R/'benchmarks/lio_compare/replay_diagnostic.py').read_text().replace("P/'common.db3'","P/'snapshot-time.db3'").replace("ROOT/'reports/lio-spike-diagnosis-20261102/raw'","ROOT/'reports/lio-spike-diagnosis-20261102/snapshot-time'")
s=s.replace("lookup={x[0]:x[1]", "availability=json.loads((P/'snapshot-availability.json').read_text())\nlookup={x[0]:x[1]")
s=s.replace("for topic_id,raw in conn.execute('select topic_id,data from messages order by id'):","for mid,topic_id,raw in conn.execute('select id,topic_id,data from messages order by id'):")
s=s.replace("dt,n=scan_info(raw);t+=dt;", "dt,n=scan_info(raw);t=availability[str(mid)];")
(R/'benchmarks/lio_compare/replay_snapshot_time.py').write_text(s)
print('Snapshot-timing intervention prepared; original arrival schedule retained.')
