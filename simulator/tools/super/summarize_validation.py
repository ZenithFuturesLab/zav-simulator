#!/usr/bin/env python3
"""Summarize a recorded flight without using truth in the control loop."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation


def summarize(source, output):
    rows=[json.loads(line) for line in Path(source).read_text().splitlines()]
    rows=[r for r in rows if 'lio_body' in r and 'truth_imu' in r]
    if not rows:raise ValueError('No paired odometry in record')
    workspace=Path(__file__).resolve().parents[3]
    sys.path.insert(0,str(workspace/'simulator/pegasus/simulator/logic/sensors'))
    from fastlivo_calibration import load_calibration
    calibration=load_calibration(workspace/'src/FAST-LIVO2/config',[.01215,0.,.11503],[0.,22.,0.])
    mount=Rotation.from_euler('ZYX',calibration['imu_orientation'],degrees=True)
    truth={r['truth_imu']['stamp']:r['truth_imu']for r in rows}
    times=np.array(sorted(truth))
    bodies=[]
    for t in times:
        record=truth[t];q=record['q'];body_r=Rotation.from_quat([q[1],q[2],q[3],q[0]])*mount.inv()
        bodies.append(np.array(record['p'])-body_r.apply(calibration['imu_position']))
    bodies=np.array(bodies)
    tracking=[r for r in rows if r.get('controller')=='TRACKING']
    track_time=np.array([r['lio_body']['stamp']for r in tracking])
    estimated=np.array([r['lio_body']['p']for r in tracking])
    physical=np.array([np.interp(track_time,times,bodies[:,i])for i in range(3)]).T
    errors=np.linalg.norm(estimated-physical,axis=1) if tracking else np.array([])
    modes=[r['mode']for r in tracking if 'mode'in r]
    report={'record':str(source),'tracking_samples':len(tracking),
            'tracking_duration_sim_s':float(np.ptp(track_time))if tracking else 0.,
            'position_error_rms_m':float(np.sqrt(np.mean(errors**2)))if len(errors)else None,
            'position_error_p95_m':float(np.percentile(errors,95))if len(errors)else None,
            'rate_only_during_tracking':bool(modes)and all(m['rates']and not m['attitude']and not m['position']and not m.get('velocity',False)and not m.get('acceleration',False)and not m.get('altitude',False)for m in modes),
            'controller_states':list(dict.fromkeys(r.get('controller')for r in rows)),
            'final_px4':rows[-1].get('px4'),'final_position':rows[-1]['lio_body']['p'],
            'landed_and_disarmed':any(r.get('controller')=='LANDED' for r in rows) and rows[-1].get('px4',{}).get('armed')==1,
            'closest_estimated_goal_distance_m':[float(np.linalg.norm(estimated-np.array(g),axis=1).min()) if tracking else None for g in ((0.,50.,1.5),(0.,-50.,1.5))]}
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    if tracking:
        flags=np.array([[int(r.get('mode',{}).get(k,False)) for k in ('rates','attitude','position','velocity','acceleration','altitude')] for r in tracking])
        values=np.column_stack((track_time,estimated,physical,errors,flags))
        np.savetxt(output.with_suffix('.csv'),values,delimiter=',',fmt='%.8g',
                   header='sim_time,lio_x,lio_y,lio_z,physical_x,physical_y,physical_z,error_m,rate_enabled,attitude_enabled,position_enabled,velocity_enabled,acceleration_enabled,altitude_enabled',comments='')
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    if tracking:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,2,figsize=(10,5))
        axes[0].plot(physical[:,0],physical[:,1],label='Physical ZAV')
        axes[0].plot(estimated[:,0],estimated[:,1],label='Selected LIO',alpha=.7)
        axes[0].set(xlabel='World X (m)',ylabel='World Y (m)',title='SUPER benchmark path')
        axes[0].axis('equal');axes[0].legend();axes[0].grid(True)
        axes[1].plot(track_time-track_time[0],errors)
        axes[1].set(xlabel='Tracking simulation time (s)',ylabel='Position error (m)',title='LIO vs. physical body origin')
        axes[1].grid(True);fig.tight_layout();fig.savefig(output.with_suffix('.png'),dpi=160);plt.close(fig)
    print(json.dumps(report,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();summarize(args.source,args.output)
