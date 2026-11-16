#!/usr/bin/env python3
"""Offline body velocity/acceleration/jerk reports; no ROS/control dependency."""
import argparse
import base64
import csv
import html
import json
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation


def kinematics(samples, source, metadata, window_s=.21):
    """Differentiate vectors on uniform time grids; never bridge gaps or resets."""
    if len(samples)<8:
        return []
    times=np.array([r['stamp'] for r in samples]);vel=np.array([r['v'] for r in samples],dtype=float)
    if source=='truth':
        quat=np.array([r['q'] for r in samples],dtype=float);omega_local=np.array([r['w']for r in samples],dtype=float)
        valid=np.isfinite(quat).all(axis=1)&(np.linalg.norm(quat,axis=1)>1e-9)&np.isfinite(omega_local).all(axis=1)
        quat[~valid]=[0,0,0,1]
        rotations=Rotation.from_quat(quat)
        mount=Rotation.from_euler('ZYX',metadata['imu_orientation'],degrees=True)
        body=rotations*mount.inv()
        lever=body.apply(metadata['imu_position'])
        omega=rotations.apply(omega_local)
        vel=rotations.apply(vel)-np.cross(omega,lever)
        vel[~valid]=np.nan
    states=np.array([r.get('state','UNKNOWN') for r in samples])
    finite=np.isfinite(times)&np.isfinite(vel).all(axis=1)
    # A missing/invalid row breaks a segment, even if timestamps would interpolate.
    breaks=np.flatnonzero((np.diff(times)<=0)|(np.diff(times)>.1)|(~finite[1:])|(~finite[:-1]))+1
    result=[]
    for idx in np.split(np.arange(len(times)),breaks):
        if len(idx)<8 or not finite[idx].all():continue
        t=times[idx];dt=float(np.median(np.diff(t)))
        if dt<=0:continue
        grid=np.arange(t[0],t[-1]+dt*.01,dt)
        n=max(5,int(round(window_s/dt))|1)
        if len(grid)<n:continue
        values=np.column_stack([np.interp(grid,t,vel[idx,i])for i in range(3)])
        # Trim both ends: no extrapolated edge maxima from the polynomial filter.
        half=n//2;inside=slice(half,-half)
        a=savgol_filter(values,n,3,deriv=1,delta=dt,axis=0)[inside]
        j=savgol_filter(values,n,3,deriv=2,delta=dt,axis=0)[inside]
        state=states[idx][np.clip(np.searchsorted(t,grid,side='right')-1,0,len(t)-1)][inside]
        result.append({'t':grid[inside],'v':values[inside],'a':a,'j':j,'state':state,'window_s':(n-1)*dt})
    return result


def peaks(parts, tracking):
    result={}
    for key,unit in (('v','m/s'),('a','m/s²'),('j','m/s³')):
        best=None
        for p in parts:
            mask=p['state']=='TRACKING' if tracking else np.ones(len(p['t']),dtype=bool)
            if not mask.any():continue
            t=p['t'][mask];values=np.linalg.norm(p[key][mask],axis=1);i=int(np.argmax(values))
            item={'value':float(values[i]),'stamp':float(t[i]),'unit':unit}
            if best is None or item['value']>best['value']:best=item
        result[key]=best
    return result


def analyze(directory, window_s=.21):
    directory=Path(directory);raw=directory/'samples.jsonl'
    metadata=json.loads((directory/'metadata.json').read_text())
    streams={'truth':[],'lio':[]};dropped=0;last_state='UNKNOWN'
    with raw.open() as f:
        for line in f:
            try:
                row=json.loads(line)
                if 'state'in row:last_state=row['state']
                if row.get('source')in streams:streams[row['source']].append(row)
            except (ValueError,KeyError):dropped+=1
    series={k:kinematics(v,k,metadata,window_s)for k,v in streams.items()}
    series={k:v for k,v in series.items()if v}
    if not series:raise ValueError('Not enough continuous odometry for a report (need >= smoothing window).')
    primary='truth' if 'truth'in series else 'lio'
    tracking=any(np.any(p['state']=='TRACKING')for p in series[primary])
    begin=min(p['t'][0]for parts in series.values()for p in parts)
    summary={'last_controller_state':last_state,'completion':metadata.get('completion','unknown'),'controller_states':list(dict.fromkeys(r.get('state','UNKNOWN')for r in streams[primary])),'primary_source':primary,'primary_scope':'TRACKING' if tracking else 'FULL_RECORD',
             'time_origin_stamp':float(begin),'time_base':'simulation seconds','velocity_frame':'world ENU, physical body origin',
             'derivatives':'Savitzky-Golay polynomial degree 3 on uniformly resampled velocity vectors; acceleration=dv/dt; jerk=d²v/dt²; gravity excluded',
             'requested_window_s':window_s,'edge_policy':'half-window trimmed at each segment end; gaps >0.1s/resets/invalid samples split segments',
             'malformed_lines_dropped':dropped,'sources':{}}
    for key,parts in series.items():
        summary['sources'][key]={'samples':len(streams[key]),'segments':len(parts),
            'effective_windows_s':[p['window_s']for p in parts],
            'tracking':peaks(parts,True),'full_record':peaks(parts,False)}
    for source_info in summary['sources'].values():
        for scope in ('tracking','full_record'):
            for item in source_info[scope].values():
                if item:item['elapsed_s']=item['stamp']-begin
    (directory/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    with (directory/'kinematics.csv').open('w',newline='')as f:
        w=csv.writer(f);w.writerow(['source','segment','sim_time_s','elapsed_s','state','vx','vy','vz','speed_mps','ax','ay','az','acc_mps2','jx','jy','jz','jerk_mps3'])
        for key,parts in series.items():
            for segment,p in enumerate(parts):
                for i,t in enumerate(p['t']):
                    vectors=[p[k][i]for k in ('v','a','j')]
                    w.writerow([key,segment,t,t-begin,p['state'][i],*[float(x)for v in vectors for x in (*v,np.linalg.norm(v))]])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    bounds=metadata.get('planner',{}).get('traj_opt',{});boundary=bounds.get('boundary',{})
    configured={'v':boundary.get('max_vel'),'a':boundary.get('max_acc'),'j':boundary.get('max_jerk')}
    jerk_on=any(bounds.get(k,{}).get('penna_jerk',-1)>0 for k in ('exp_traj','backup_traj'))
    for filename,scope in (('motion.png',tracking),('full_flight.png',False)):
        fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
        for ax,key,label in zip(axes,('v','a','j'),('Speed (m/s)','Acceleration (m/s²)','Jerk (m/s³)')):
            for source,parts in series.items():
                for index,p in enumerate(parts):
                    values=np.linalg.norm(p[key],axis=1);mask=p['state']=='TRACKING' if scope else np.ones(len(values),dtype=bool)
                    ax.plot(p['t']-begin,np.where(mask,values,np.nan),label=('Physical body' if source=='truth' else 'LIO estimate')if index==0 else None,
                            color='tab:blue' if source=='truth' else 'tab:orange',alpha=1 if source==primary else .5,lw=1.1)
            peak=peaks(series[primary],scope)[key]
            if peak:
                x=peak['stamp']-begin;y=peak['value'];ax.plot(x,y,'ro',ms=5)
                ax.annotate(f'MAX {y:.3f} {peak["unit"]}  @ {x:.2f}s',xy=(x,y),xytext=(.02,.92),textcoords='axes fraction',va='top',color='darkred',bbox={'facecolor':'white','alpha':.85,'edgecolor':'none'},arrowprops={'arrowstyle':'->','color':'darkred'})
            limit=configured[key]
            if limit is not None and (key!='j' or jerk_on):ax.axhline(limit,color='gray',ls='--',lw=1,label=f'Configured boundary {limit:g} (soft)')
            if key=='j' and not jerk_on:ax.text(.02,.04,'Configured jerk penalty is disabled',transform=ax.transAxes,ha='left',va='bottom',fontsize=9)
            ax.set_ylabel(label);ax.set_ylim(bottom=0);ax.grid(alpha=.25);ax.legend(loc='upper right',fontsize=8)
        axes[-1].set_xlabel('Elapsed simulation time (s)')
        fig.suptitle(f'{metadata.get("scenario","")} / {metadata.get("backend","")} — {"TRACKING only" if scope else "Full recording (includes takeoff/landing)"}\nVector norms; acceleration/jerk smoothed over ~{window_s:.2f}s; maxima from '+('physical truth' if primary=='truth' else 'LIO estimate'))
        fig.tight_layout(rect=(0,0,1,.95));fig.savefig(directory/filename,dpi=150);plt.close(fig)
    chosen=summary['sources'][primary]['tracking' if tracking else 'full_record']
    def cards(values):
        return ''.join(f'<div class="card"><span>{label}</span><strong>{values[k]["value"]:.3f} <small>{values[k]["unit"]}</small></strong><span>t = {values[k]["stamp"]-begin:.2f} s</span></div>' for k,label in (('v','최대 속도'),('a','최대 가속도'),('j','최대 저크'))if values[k])
    def embedded(name):return 'data:image/png;base64,'+base64.b64encode((directory/name).read_bytes()).decode()
    title=html.escape(f'{metadata.get("scenario","")} / {metadata.get("backend","")}')
    report=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>SUPER 주행 데이터</title>
<style>body{{font:16px system-ui,sans-serif;background:#f3f5f8;color:#172234;max-width:1200px;margin:36px auto;padding:0 20px}}h1{{margin-bottom:8px}}.cards{{display:flex;gap:16px;flex-wrap:wrap}}.card{{background:white;border-radius:12px;padding:20px;flex:1;min-width:200px;display:grid;gap:10px}}strong{{font-size:30px}}small{{font-size:16px}}img{{width:100%;background:white;border-radius:12px;margin:20px 0}}p{{line-height:1.7}}a{{color:#245acc}}</style>
<h1>SUPER 주행 데이터 · {title}</h1><p>{html.escape(metadata.get('created_at',''))}<br>
기록 종료: {html.escape(metadata.get('completion','unknown'))} · 최종 제어 상태: {html.escape(last_state)}<br>기준: {'Isaac 실제 기체 body' if primary=='truth' else 'LIO 추정값 (실제 기체 데이터 없음)'} · {'주행 TRACKING 구간' if tracking else '전체 기록 (TRACKING 구간 없음)'} · 시간: 시뮬레이션 초</p>
<div class="cards">{cards(chosen)}</div><p>속도·가속도·저크는 3차원 벡터의 크기입니다. 가속도는 중력을 제외한 이동 가속도입니다.
가속도·저크는 속도 벡터를 약 {window_s:.2f}초 평활 미분한 값이며, 필터 폭에 따라 피크가 달라집니다. 각 연속 구간 양끝 반 윈도는 제외합니다.
주황색은 LIO 추정값으로, 미분 과정에서 추정 오차가 확대될 수 있습니다. 회색 점선은 설정값이며 실제 기체의 엄격한 상한을 뜻하지 않습니다.</p>
<img src="{embedded('motion.png')}" alt="주행 속도 가속도 저크 및 최댓값"><details><summary>이륙·착륙 포함 전체 기록 최댓값 보기</summary><p>착륙 접촉 피크도 포함하므로 주행 최대값과 구분하세요.</p><div class="cards">{cards(summary['sources'][primary]['full_record'])}</div><img src="{embedded('full_flight.png')}" alt="전체 비행 그래프"></details>
<p><a href="kinematics.csv">CSV 내려받기</a> · <a href="summary.json">최댓값 JSON</a> · <a href="samples.jsonl">원본 기록</a></p></html>'''
    (directory/'report.html').write_text(report)
    print('[SUPER telemetry] '+str(directory/'report.html'),flush=True)
    print('[SUPER telemetry] '+', '.join(f'{k}_max={v["value"]:.3f} {v["unit"]}' for k,v in chosen.items()if v),flush=True)
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--window',type=float,default=.21);a=p.parse_args()
    if not np.isfinite(a.window) or a.window<=0:p.error('--window must be positive and finite')
    analyze(a.directory,a.window)

if __name__=='__main__':main()
