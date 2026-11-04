from pathlib import Path
import json,sqlite3,struct
import numpy as np
R=Path(__file__).resolve().parents[2];P=R/'.cache/lio-comparison-20261102';O=R/'reports/lio-spike-diagnosis-20261102';a=sqlite3.connect(P/'common.db3');b=sqlite3.connect(P/'snapshot-time.db3');availability=json.loads((P/'snapshot-availability.json').read_text());lookup=dict(a.execute('select id,name from topics'));imu=lidar=0
for (mid,topic,raw),(mi2,to2,new) in zip(a.execute('select id,topic_id,data from messages order by id'),b.execute('select id,topic_id,data from messages order by id')):
 assert mid==mi2 and topic==to2
 if lookup[topic].endswith('/imu'):assert raw==new;imu+=1;continue
 n,=struct.unpack_from('<I',raw,12);pos=4+((12+n+7)//8)*8;count,=struct.unpack_from('<I',raw,pos+16);old=np.frombuffer(raw,dtype='u1',offset=pos+20).reshape(count,20);changed=np.frombuffer(new,dtype='u1',offset=pos+20).reshape(count,20);assert np.array_equal(old[:,4:],changed[:,4:])
 sec,ns=struct.unpack_from('<iI',raw,4);offsets=np.ndarray((count,),dtype='<u4',buffer=raw,offset=pos+20,strides=(20,));assert availability[str(mid)]==sec*10**9+ns+int(offsets.max());lidar+=1
runs=[]
for group in ['raw','frame-intervention','no-deskew','oracle-deskew','boundary-fix','snapshot-time']:
 for p in (O/group).glob('*/run.json'):
  run=json.loads(p.read_text());runs.append(dict(group=group,algorithm=run['algorithm'],input_counts=run['input_counts'],first_stamp=run['first_stamp'],last_stamp=run['last_stamp']))
for r in runs:assert r['input_counts']=={'imu':16146,'lidar':801}
z=dict(passed=True,identical_imu_records=imu,identical_lidar_xyz_intensity_tag_line_records=lidar,original_lidar_availability_preserved=True,runs=runs);(O/'verification.json').write_text(json.dumps(z,indent=2));print('PASS:',imu,'identical IMU messages;',lidar,'identical XYZ sets; arrival schedule preserved;',len(runs),'replays')
