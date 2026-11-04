#!/usr/bin/env python3
"""Check recorded IMU against direct PhysX GT; diagnostic only, not estimator benchmark."""
from pathlib import Path
import json,sqlite3
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from scipy.signal import savgol_filter
import rclpy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
R=Path(__file__).resolve().parents[2];P=R/'.cache/lio-comparison-20261102';O=R/'reports/lio-spike-diagnosis-20261102'
a=[json.loads(l) for p in sorted(P.glob('truth-*.jsonl')) for l in p.read_text().splitlines()]
t=np.array([x['t'] for x in a]);pos=np.array([x['p'] for x in a]);rot=Rotation.from_quat([x['q'] for x in a]);sl=Slerp(t,rot)
c=sqlite3.connect(next((P/'dataset').glob('*.db3')));m=[deserialize_message(raw,Imu) for raw, in c.execute('select data from messages where topic_id=(select id from topics where name like "%/imu") order by id')]
ti=np.array([x.header.stamp.sec+x.header.stamp.nanosec*1e-9 for x in m]);acc=np.array([[x.linear_acceleration.x,x.linear_acceleration.y,x.linear_acceleration.z] for x in m]);gyro=np.array([[x.angular_velocity.x,x.angular_velocity.y,x.angular_velocity.z] for x in m]);keep=(ti>1)&(ti<125);ti=ti[keep];acc=acc[keep];gyro=gyro[keep]
# Compute physical angular velocity in world coordinates over each GT interval.
mid=(t[:-1]+t[1:])/2;ww=rot[:-1].apply((rot[:-1].inv()*rot[1:]).as_rotvec()/np.diff(t)[:,None])
a_gt=savgol_filter(pos,9,3,deriv=2,delta=float(np.median(np.diff(t))),axis=0)
results=[]
for lag in [-.01,-.005,0,.005,.01]:
 tt=ti+lag;rr=sl(tt);world_acc=rr.apply(acc)+np.array([0,0,-9.80665]);truth_acc=np.column_stack([np.interp(tt,t,a_gt[:,j]) for j in range(3)]);world_gyro=rr.apply(gyro);truth_gyro=np.column_stack([np.interp(tt,mid,ww[:,j]) for j in range(3)]);ea=np.linalg.norm(world_acc-truth_acc,axis=1);eg=np.rad2deg(np.linalg.norm(world_gyro-truth_gyro,axis=1))
 for lo,hi in [(40,50),(51,52),(58,60),(65.7,67.5),(73.25,74.6)]:
  mask=(ti>=lo)&(ti<hi);results.append(dict(lag_s=lag,window=[lo,hi],acc_rmse=float(np.sqrt(np.mean(ea[mask]**2))),gyro_rmse_deg_s=float(np.sqrt(np.mean(eg[mask]**2))),acc_peak=float(ea[mask].max()),gyro_peak_deg_s=float(eg[mask].max())))
 if lag==0:np.savez(O/'imu_consistency.npz',t=ti,acc_error=ea,gyro_error=eg,world_acc=world_acc,truth_acc=truth_acc,world_gyro=world_gyro,truth_gyro=truth_gyro)
(O/'imu_consistency.json').write_text(json.dumps(results,indent=2));print(json.dumps([x for x in results if x['lag_s']==0],indent=2))
