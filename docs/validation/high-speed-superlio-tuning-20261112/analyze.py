"""Independent physical clearance and timestamp-aligned odometry diagnostics."""
import json,sys,re
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
WS=Path(__file__).resolve().parents[3];sys.path.insert(0,str(WS/'simulator/tools/super'));from pcd_scene import read_pcd
sys.path.insert(0,str(WS/'simulator/pegasus/simulator/logic/sensors'));from fastlivo_calibration import load_calibration
cal=load_calibration(WS/'src/FAST-LIVO2/config',[.01215,0,.11503],[0,22,0]);mount=Rotation.from_euler('ZYX',cal['imu_orientation'],degrees=True)
points=read_pcd(WS/'.cache/SUPER-official/mars_uav_sim/perfect_drone_sim/pcd/random_map_24_6635.pcd');tree=cKDTree(points[np.abs(points[:,2])>1e-7])
for name in sys.argv[1:]:
    folder=Path(name);rows=[json.loads(l) for l in (folder/'flight.jsonl').read_text().splitlines()];track=[r for r in rows if r.get('controller')=='TRACKING' and 'lio_body' in r];
    if not track:print(folder.name,'no tracking');continue
    begin=track[0]['lio_body']['stamp'];end=track[-1]['lio_body']['stamp']
    ev=folder/'odom-events.jsonl'
    events=[json.loads(l) for l in ev.read_text().splitlines()] if ev.exists() else [dict(key=k,**r[k]) for r in rows for k in ('truth_imu','lio_body') if k in r]
    truth={r['stamp']:r for r in events if r['key']=='truth_imu'};tt=np.array(sorted(truth));tr=[truth[t] for t in tt]
    rot=Rotation.from_quat(np.array([r['q'] for r in tr])[:,[1,2,3,0]])*mount.inv();body=np.array([r['p'] for r in tr])-rot.apply(cal['imu_position'])
    lio={r['stamp']:r for r in events if r['key']=='lio_body' and begin<=r['stamp']<=end};lt=np.array(sorted(lio));lp=np.array([lio[t]['p'] for t in lt]);actual=np.column_stack([np.interp(lt,tt,body[:,i]) for i in range(3)]);err=np.linalg.norm(lp-actual,axis=1)
    mask=(tt>=begin)&(tt<=end);path=body[mask];pt=tt[mask];segments=[]
    for p,q in zip(path[:-1],path[1:]):segments.append(p+np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(q-p)/.01))+1))[:,None]*(q-p))
    closest=float(tree.query(np.concatenate(segments))[0].min());speed=np.linalg.norm(np.diff(path,axis=0)/np.diff(pt)[:,None],axis=1)
    gap=np.diff(lt);rx=np.array([lio[t].get('rx',np.nan) for t in lt]);rgap=np.diff(rx)
    refs=[];ref_vel=[];ref_acc=[];ref_jerk=[];tracking_err=[]
    for row in track:
        trajectory=row.get('trajectory')
        if trajectory is None:continue
        t=row['lio_body']['stamp']-trajectory['start']
        if t<0:continue
        durations=trajectory['durations'];idx=0
        while idx<len(durations)-1 and t>durations[idx]:t-=durations[idx];idx+=1
        t=min(t,durations[idx]);coefs=[np.array(trajectory[k][idx*8:idx*8+8]) for k in ('x','y','z')]
        ref=np.array([np.polyval(c,t) for c in coefs]);refs.append(ref)
        tracking_err.append(float(np.linalg.norm(ref-np.array(row['lio_body']['p']))))
        for degree,values in ((1,ref_vel),(2,ref_acc),(3,ref_jerk)):
            values.append(float(np.linalg.norm([np.polyval(np.polyder(c,degree),t) for c in coefs])))
    log=(folder/'launch.log').read_text(errors='replace');complete='Benchmark complete' in log
    report={'trial':folder.name,'completed':complete,'landed':any(r.get('controller')=='LANDED' and r.get('px4',{}).get('armed')==1 for r in rows),'faults':list(dict.fromkeys(r.get('controller') for r in rows if r.get('controller','').startswith(('FAULT','STOPPED')))), 'physical_goal_min_distance_m':[float(np.linalg.norm(path-np.array((0,50,1.5)),axis=1).min()),float(np.linalg.norm(path[int(np.argmax(path[:,1])):]-np.array((0,-50,1.5)),axis=1).min())], 'tracking_sim_s':end-begin,'tracking_wall_s':track[-1]['wall_elapsed']-track[0]['wall_elapsed'],'physical_peak_speed_mps':float(speed.max()),'physical_p95_speed_mps':float(np.percentile(speed,95)),'path_length_m':float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),'lio_rms_m':float(np.sqrt(np.mean(err**2))),'lio_p95_m':float(np.percentile(err,95)),'lio_max_m':float(err.max()),'nearest_obstacle_surface_sample_m':closest,'conservative_body_clearance_m':closest-.086602541-.005-.33,'odom_sim_hz':float(1/np.mean(gap)),'odom_max_sim_gap_s':float(gap.max()),'odom_max_wall_gap_s':float(np.nanmax(rgap)) if np.isfinite(rgap).any() else None,'odom_samples':len(lt)}
    report.update(reference_peak_speed_mps=max(ref_vel,default=None),reference_peak_acc_mps2=max(ref_acc,default=None),reference_peak_jerk_mps3=max(ref_jerk,default=None),tracking_error_rms_m=float(np.sqrt(np.mean(np.square(tracking_err)))) if tracking_err else None,tracking_error_max_m=max(tracking_err,default=None))
    active=[r for r in rows if r.get('controller') in ('TAKEOFF','READY','TRACKING','LANDING','LANDED') and 'lio_body' in r]
    first_active=active[0]['lio_body']['stamp'];last_active=active[-1]['lio_body']['stamp']
    all_lio={r['stamp']:r for r in events if r['key']=='lio_body' and max(first_active,tt[0])<=r['stamp']<=min(last_active,tt[-1])}
    all_t=np.array(sorted(all_lio));all_p=np.array([all_lio[t]['p']for t in all_t]);all_actual=np.column_stack([np.interp(all_t,tt,body[:,i])for i in range(3)]);all_error=np.linalg.norm(all_p-all_actual,axis=1)
    report.update(flight_lio_rms_m=float(np.sqrt(np.mean(all_error**2))),flight_lio_max_m=float(all_error.max()),final_physical_body_position_m=body[-1].tolist())
    modes=[r['mode']for r in track if 'mode'in r]
    report['rate_only_during_tracking']=bool(modes) and all(m.get('rates',False) and not any(m.get(k,False) for k in ('attitude','position','velocity','acceleration','altitude')) for m in modes)
    report['accepted']=bool(report['rate_only_during_tracking'] and report['completed'] and report['landed'] and not report['faults'] and report['flight_lio_max_m']<=.5 and report['flight_lio_rms_m']<=.25 and report['conservative_body_clearance_m']>=.1 and max(report['physical_goal_min_distance_m'])<=.8 and report['odom_max_sim_gap_s']<=.05 and report['odom_max_wall_gap_s'] is not None and report['odom_max_wall_gap_s']<=.2)
    (folder/'metrics.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
