#!/usr/bin/env python3
from pathlib import Path
import json,sqlite3
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
R=Path(__file__).resolve().parents[2];P=R/'.cache/lio-fixed-20261102';O=R/'reports/lio-fixed-20261102';a=[json.loads(l) for f in sorted(P.glob('truth-*.jsonl')) for l in f.read_text().splitlines()];t=np.array([x['t'] for x in a]);rot=Rotation.from_quat([x['q'] for x in a]);sl=Slerp(t,rot);w=np.array([x['body_w'] for x in a]);c=sqlite3.connect(next((P/'dataset').glob('*.db3')));imu=[deserialize_message(raw,Imu) for raw, in c.execute('select data from messages where topic_id=(select id from topics where name like "%/imu") order by id')];ti=np.array([x.header.stamp.sec+x.header.stamp.nanosec*1e-9 for x in imu]);g=np.array([[x.angular_velocity.x,x.angular_velocity.y,x.angular_velocity.z] for x in imu]);mask=(ti>t.min())&(ti<t.max());tt=ti[mask];gyro_world=sl(tt).apply(g[mask]);truth_world=np.column_stack([np.interp(tt,t,w[:,i]) for i in range(3)]);error=np.rad2deg(np.linalg.norm(gyro_world-truth_world,axis=1));window=(tt>51)&(tt<79);dt=np.diff(ti)
r=dict(runtime=json.loads((P/'runtime-settings.json').read_text()),imu_messages=len(imu),gt_samples=len(t),imu_gaps_over_7_5_ms=int((dt>.0075).sum()),imu_max_gap_ms=float(dt.max()*1000),gyro_world_rmse_deg_s=float(np.sqrt(np.mean(error[window]**2))),gyro_world_peak_error_deg_s=float(error[window].max()),gt_source='Direct PhysX rigid-body pose and world angular velocity, no EKF state access')
assert r['runtime']['motion_bvh'];assert r['runtime']['sensor_motion_state']=='NONCOMPENSATED';assert r['gyro_world_rmse_deg_s']<.05,r
(O/'sensor_validation.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
