#!/usr/bin/env python3
from pathlib import Path
import json,csv,sqlite3,struct
import numpy as np
R=Path(__file__).resolve().parents[2];P=R/'.cache/lio-fixed-20261102';O=R/'reports/lio-fixed-20261102'
f=[json.loads(s) for s in (P/'capture.jsonl').read_text().splitlines()];lo=min(x['truth']['sim'] for x in f if x['stage']=='north_1m');hi=min(x['truth']['sim'] for x in f if x['stage']=='hover_final');c=sqlite3.connect(P/'common.db3');expected=[]
for raw, in c.execute('select data from messages where topic_id=(select id from topics where name like "%/lidar")'):
 sec,ns=struct.unpack_from('<iI',raw,4);n,=struct.unpack_from('<I',raw,12);pos=4+((12+n+7)//8)*8;count,=struct.unpack_from('<I',raw,pos+16);dt=np.ndarray((count,),dtype='<u4',buffer=raw,offset=pos+20,strides=(20,));end=sec+ns*1e-9+int(dt.max())*1e-9
 if lo<=end<=hi:expected.append((end,count))
expected=np.array(expected);runs=[];inputs=[];schedules=[]
for p in sorted((O/'raw').glob('*/run.json')):
 r=json.loads(p.read_text());inputs.append(r['input_counts']);schedules.append((p.parent/'input_availability.json').read_bytes());a=[x for x in csv.DictReader((p.parent/'stages.csv').open()) if lo<=float(x['stamp'])<=hi and int(x['input_points'])>0];t=np.array([float(x['stamp']) for x in a]);n=np.array([int(x['selected_points']) for x in a]);incoming=np.array([int(x['input_points']) for x in a]);assert len(a)==len(expected),(p,len(a),len(expected));assert np.max(abs(t-expected[:,0]))<1e-5;assert np.array_equal(n,expected[:,1]),p;assert np.array_equal(n,incoming),p
 runs.append(dict(run=p.parent.name,frames=len(a),mean_selected=float(n.mean()),max_timestamp_delta_s=float(np.max(abs(t-expected[:,0])))))
assert len(runs)==6,len(runs);assert all(x==inputs[0] for x in inputs);assert all(x==schedules[0] for x in schedules)
settings=json.loads((P/'runtime-settings.json').read_text());assert settings['lidar_time_contract']=='GMO frameStart-coherent snapshots';assert settings['motion_bvh'];assert settings['physics_dt']==.005
assert 'benchmark_record' not in (R/'simulator/zav_sim.py').read_text()
result=dict(passed=True,input_counts=inputs[0],identical_raw_returns=True,identical_input_availability=True,runs=runs);(O/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
