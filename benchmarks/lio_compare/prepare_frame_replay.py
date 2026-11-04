#!/usr/bin/env python3
"""Diagnostic intervention: undo stale render attitude in recorded IMU, using recorded PhysX attitude.
GT-assisted diagnosis, NOT a deployable estimator or fair accuracy benchmark.
"""
from pathlib import Path
import json,sqlite3
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from rclpy.serialization import deserialize_message,serialize_message
from sensor_msgs.msg import Imu
R=Path(__file__).resolve().parents[2];P=R/'.cache/lio-comparison-20261102';O=R/'reports/lio-spike-diagnosis-20261102'
a=[json.loads(l) for f in sorted(P.glob('truth-*.jsonl')) for l in f.read_text().splitlines()];t=np.array([x['t'] for x in a]);rot=Rotation.from_quat([x['body_q'] for x in a]);sl=Slerp(t,rot);mount=rot[0].inv()*Rotation.from_quat(a[0]['q']);offset=rot[0].inv().apply(np.array(a[0]['p'])-a[0]['body_p'])
c=sqlite3.connect(P/'common.db3');rows=c.execute('select id,data from messages where topic_id=(select id from topics where name like "%/imu") order by id').fetchall();m=[deserialize_message(raw,Imu) for _,raw in rows];ti=np.array([x.header.stamp.sec+x.header.stamp.nanosec*1e-9 for x in m]);gyro=np.array([[x.angular_velocity.x,x.angular_velocity.y,x.angular_velocity.z] for x in m]);acc=np.array([[x.linear_acceleration.x,x.linear_acceleration.y,x.linear_acceleration.z] for x in m]);body_g=mount.apply(gyro)
mid=(t[:-1]+t[1:])/2;ww=rot[:-1].apply((rot[:-1].inv()*rot[1:]).as_rotvec()/np.diff(t)[:,None]);evalmask=(ti>51)&(ti<80.75)
truth_w=np.column_stack([np.interp(ti-.0025,mid,ww[:,i]) for i in range(3)])
phases=[]
for phase in np.arange(0,.05,.005):
 stale_time=phase+np.floor((ti-phase-1e-7)/.05)*.05;stale=sl(np.clip(stale_time,t.min(),t.max()));error=np.linalg.norm(stale.apply(body_g)-truth_w,axis=1);phases.append(dict(phase=float(phase),gyro_world_rmse_deg_s=float(np.rad2deg(np.sqrt(np.mean(error[evalmask]**2))))))
# Phase 0.01 s is fixed by recorded render frames, not tuned using odometry error.
phase=.01;stale_time=phase+np.floor((ti-phase-1e-7)/.05)*.05;stale=sl(np.clip(stale_time,t.min(),t.max()));actual=sl(np.clip(ti,t.min(),t.max()));omega_world=stale.apply(body_g);true_body_g=actual.inv().apply(omega_world)
dt=np.diff(ti);alpha_old=np.vstack([np.zeros(3),np.diff(body_g,axis=0)/dt[:,None]]);alpha_new=np.vstack([np.zeros(3),np.diff(true_body_g,axis=0)/dt[:,None]])
base_old=mount.apply(acc)-np.cross(alpha_old,offset)-np.cross(body_g,np.cross(body_g,offset));specific_world=stale.apply(base_old);base_new=actual.inv().apply(specific_world)+np.cross(alpha_new,offset)+np.cross(true_body_g,np.cross(true_body_g,offset));newacc=mount.inv().apply(base_new);newgyro=mount.inv().apply(true_body_g)
# Preserve initial values lacking a matching GT sample; they are stationary.
newacc[ti<t.min()]=acc[ti<t.min()];newgyro[ti<t.min()]=gyro[ti<t.min()]
out=P/'frame-fixed-diagnostic.db3';assert not out.exists();d=sqlite3.connect(out);c.backup(d)
for (mid_,raw),msg,aa,gg in zip(rows,m,newacc,newgyro):
 for j,key in enumerate('xyz'):setattr(msg.linear_acceleration,key,float(aa[j]));setattr(msg.angular_velocity,key,float(gg[j]))
 d.execute('update messages set data=? where id=?',(serialize_message(msg),mid_))
d.commit();d.close();c.close();(O/'render_phase_check.json').write_text(json.dumps(phases,indent=2));np.savez(O/'frame_intervention.npz',t=ti,oldacc=acc,newacc=newacc,oldgyro=gyro,newgyro=newgyro,stale_t=stale_time)
print(json.dumps(phases,indent=2))
s=(R/'benchmarks/lio_compare/replay_diagnostic.py').read_text().replace("P/'common.db3'","P/'frame-fixed-diagnostic.db3'").replace("ROOT/'reports/lio-spike-diagnosis-20261102/raw'","ROOT/'reports/lio-spike-diagnosis-20261102/frame-intervention'")
(R/'benchmarks/lio_compare/replay_frame_intervention.py').write_text(s)
