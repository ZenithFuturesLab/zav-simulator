#!/usr/bin/env python3
import json,math
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
ROOT=Path(__file__).resolve().parents[2];P=ROOT/'.cache/lio-comparison-20261102';OUT=ROOT/'reports/lio-comparison-20261102'
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'figure.dpi':140})
NAMES={'super_lio':'Super-LIO','fast_livo':'FAST-LIVO2 (LIO only)'}
COLORS={'super_lio':'#136f63','fast_livo':'#d2691e'}
truth=[json.loads(s) for f in sorted(P.glob('truth-*.jsonl')) for s in f.read_text().splitlines()]
gt=pd.DataFrame([dict(t=x['t'],**{f'p{i}':x['p'][i] for i in range(3)},**{f'q{i}':x['q'][i] for i in range(4)}) for x in truth]).drop_duplicates('t').sort_values('t')
gt_t=gt.t.to_numpy();gt_p=gt[[f'p{i}' for i in range(3)]].to_numpy();gt_rot=Rotation.from_quat(gt[[f'q{i}' for i in range(4)]].to_numpy());gt_slerp=Slerp(gt_t,gt_rot)
flight=[json.loads(s) for s in (P/'capture.jsonl').read_text().splitlines()]
lo=min(x['truth']['sim'] for x in flight if x['stage']=='takeoff');hi=min(x['truth']['sim'] for x in flight if x['stage']=='landing');grid=np.arange(math.ceil(lo*10)/10,hi,.1)
GTp=np.column_stack([np.interp(grid,gt_t,gt_p[:,i]) for i in range(3)]);GTr=gt_slerp(grid)
rows=[];series={};coverage=[]
def trajectory(source,kind,common_mask=None):
 a=[x for x in source if x['kind']==kind and np.isfinite(x['p']).all() and np.isfinite(x['q']).all()]
 # First publication of each measurement time, so delayed re-publications cannot improve past accuracy.
 d={}
 for x in a:d.setdefault(x['t'],x)
 a=sorted(d.values(),key=lambda x:x['t']);t=np.array([x['t'] for x in a]);p=np.array([x['p'] for x in a]);q=np.array([x['q'] for x in a]);rot=Rotation.from_quat(q)
 init=(t>=5)&(t<=15);assert init.sum()>=20
 # Initial stationary pose alignment only; no scale or whole-trajectory fitting.
 rt=gt_slerp(t[init]);R=rt.mean()*rot[init].mean().inv()
 trans=np.column_stack([np.interp(t[init],gt_t,gt_p[:,i]) for i in range(3)]).mean(axis=0)-R.apply(p[init]).mean(axis=0)
 p=R.apply(p)+trans;rot=R*rot
 indices=np.clip(np.searchsorted(t,grid),1,len(t)-1)
 valid=(grid>=t.min())&(grid<=t.max())&((t[indices]-t[indices-1])<=(.2 if kind=='lidar' else .03))
 if common_mask is not None:valid &= common_mask
 tt=grid[valid]
 pp=np.column_stack([np.interp(tt,t,p[:,i]) for i in range(3)]);rr=Slerp(t,rot)(tt)
 return tt,pp,rr,valid,len(a)
source_cache={};shared_masks={}
for folder in sorted((OUT/'raw').glob('*')):
 if not (folder/'run.json').exists():continue
 run=json.loads((folder/'run.json').read_text());source=[json.loads(s) for s in (folder/'odometry.jsonl').read_text().splitlines()];source_cache[folder]=source
 for kind in ['lidar','imu']:
  _,_,_,mask,_=trajectory(source,kind);key=(run['profile'],kind)
  shared_masks[key]=mask if key not in shared_masks else shared_masks[key]&mask
for folder in sorted(source_cache):
 run=json.loads((folder/'run.json').read_text());alg=run['algorithm'];profile=run['profile'];trial=run['trial']
 stages=pd.read_csv(folder/'stages.csv');stages=stages[(stages.stamp>=lo)&(stages.stamp<=hi)&(stages.input_points>0)]
 resources=pd.DataFrame(json.loads(s) for s in (folder/'resources.jsonl').read_text().splitlines());resources['cpu_percent']=(resources.cpu_user.diff()+resources.cpu_system.diff())/resources.wall.diff()*100
 r=resources[(resources.sim>=lo)&(resources.sim<=hi)]
 cpu_seconds=(r.iloc[-1].cpu_user+r.iloc[-1].cpu_system)-(r.iloc[0].cpu_user+r.iloc[0].cpu_system)
 cpu_percent=cpu_seconds/(r.iloc[-1].wall-r.iloc[0].wall)*100
 base=dict(whole_cpu_ms_per_scan=cpu_seconds*1000/len(stages),algorithm=alg,label=NAMES[alg],profile=profile,trial=trial,frames=len(stages),input_points=stages.input_points.mean(),selected_points=stages.selected_points.mean(),cpu_percent=cpu_percent,peak_rss_mib=r.rss.max()/2**20,mean_rss_mib=r.rss.mean()/2**20,total_ms=stages.total_ms.mean(),p95_ms=stages.total_ms.quantile(.95),frame_cpu_ms=stages.process_cpu_ms.mean())
 for name in ['imu_undistort','downsample','map_prepare','estimate','map_update','output_other']:base[name+'_ms']=stages[name+'_ms'].mean()
 source=source_cache[folder]
 for kind in ['lidar','imu']:
  tt,pp,rr,valid,n=trajectory(source,kind,shared_masks[(profile,kind)]);target=GTp[valid];rotation=GTr[valid];ep=np.linalg.norm(pp-target,axis=1);er=np.rad2deg((rotation.inv()*rr).magnitude())
  # One-second relative pose error, same temporal sampling for both algorithms.
  lag=10;rel_gt=rotation[:-lag].inv().apply(target[lag:]-target[:-lag]);rel_est=rr[:-lag].inv().apply(pp[lag:]-pp[:-lag]);rpe=np.linalg.norm(rel_est-rel_gt,axis=1);rpe=rpe[np.abs((tt[lag:]-tt[:-lag])-1.)<1e-5]
  axis_rmse=np.sqrt(np.mean((pp-target)**2,axis=0))
  metrics=dict(rmse_x_m=axis_rmse[0],rmse_y_m=axis_rmse[1],rmse_z_m=axis_rmse[2],ate_rmse_m=np.sqrt(np.mean(ep**2)),ate_mae_m=ep.mean(),ate_p95_m=np.quantile(ep,.95),ate_max_m=ep.max(),rotation_rmse_deg=np.sqrt(np.mean(er**2)),rotation_p95_deg=np.quantile(er,.95),rpe_1s_rmse_m=np.sqrt(np.mean(rpe**2)),coverage_percent=valid.mean()*100,odom_samples=n)
  rows.append(dict(base,kind=kind,**metrics))
  if profile=='deployed' and trial==1:series[(alg,kind)]=dict(t=tt,p=pp,error=ep,angle=er)
  pd.DataFrame(dict(t=tt,error_m=ep,dx_m=pp[:,0]-target[:,0],dy_m=pp[:,1]-target[:,1],dz_m=pp[:,2]-target[:,2],rotation_error_deg=er)).to_csv(folder/f'errors-{kind}.csv',index=False)
metrics=pd.DataFrame(rows);metrics.to_csv(OUT/'all_trials.csv',index=False)
if metrics.empty:raise RuntimeError('No completed trials')
metrics.groupby(['profile','algorithm','kind'])[metrics.select_dtypes(include='number').columns].agg(['mean','std']).to_csv(OUT/'summary_full.csv')
# Summary per algorithm: process metrics once and accuracy for both outputs.
summary=[]
for (profile,alg),group in metrics.groupby(['profile','algorithm']):
 a=group[group.kind=='imu'];b=group[group.kind=='lidar'];row=dict(profile=profile,algorithm=NAMES[alg],trials=len(a))
 for column in ['total_ms','p95_ms','cpu_percent','peak_rss_mib','selected_points','frame_cpu_ms','whole_cpu_ms_per_scan']:
  row[column]=a[column].mean();row[column+'_std']=a[column].std(ddof=1)
 for kind,data in [('imu',a),('lidar',b)]:
  for column in ['ate_rmse_m','ate_p95_m','ate_max_m','rmse_x_m','rmse_y_m','rmse_z_m','rotation_rmse_deg','rpe_1s_rmse_m','coverage_percent']:row[kind+'_'+column]=data[column].mean()
 summary.append(row)
summary=pd.DataFrame(summary);summary.to_csv(OUT/'comparison.csv',index=False)
print(summary.to_string(index=False))
if len(series)<4:raise SystemExit(0)
pdf=PdfPages(OUT/'comparison_report.pdf')
fig,axes=plt.subplots(2,2,figsize=(13,9),constrained_layout=True)
for alg in ['super_lio','fast_livo']:
 s=series[(alg,'imu')];c=COLORS[alg];axes[0,0].plot(s['p'][:,0],s['p'][:,1],label=NAMES[alg],color=c,lw=1.3);axes[0,1].plot(s['t'],s['error']*100,color=c,label=NAMES[alg]);axes[1,0].plot(s['t'],s['angle'],color=c,label=NAMES[alg]);axes[1,1].plot(s['t'],s['p'][:,2],color=c,label=NAMES[alg])
axes[0,0].plot(GTp[:,0],GTp[:,1],'k--',label='PhysX ground truth',lw=1.5);axes[1,1].plot(grid,GTp[:,2],'k--',label='PhysX ground truth')
for ax in axes.flat:ax.grid(alpha=.25);ax.legend(fontsize=8)
axes[0,0].set(title='Trajectory (initial pose alignment only)',xlabel='Isaac world X [m]',ylabel='Isaac world Y [m]',aspect='equal');axes[0,1].set(title='IMU-propagated odometry position error',xlabel='Simulation time [s]',ylabel='3D error [cm]');axes[1,0].set(title='Orientation error',xlabel='Simulation time [s]',ylabel='Geodesic angle [deg]');axes[1,1].set(title='Altitude at mounted IMU',xlabel='Simulation time [s]',ylabel='Z [m]')
fig.savefig(OUT/'accuracy.png');fig.savefig(OUT/'accuracy.pdf');pdf.savefig(fig);plt.close(fig)
fig,axes=plt.subplots(2,2,figsize=(13,9),constrained_layout=True)
labels=[NAMES[x] for x in ['super_lio','fast_livo']];algos=['super_lio','fast_livo'];x=np.arange(2)
base=metrics[(metrics.kind=='imu')&(metrics.profile=='deployed')]
for ax,col,title,unit in [(axes[0,0],'total_ms','Per-scan processing time','ms / scan'),(axes[0,1],'cpu_percent','Whole mapper process CPU','% (one logical core = 100%)'),(axes[1,1],'peak_rss_mib','Peak resident memory','MiB')]:
 means=[base[base.algorithm==a][col].mean() for a in algos];std=[base[base.algorithm==a][col].std(ddof=1) for a in algos];bars=ax.bar(x,means,yerr=np.nan_to_num(std),color=[COLORS[a] for a in algos],capsize=4);ax.bar_label(bars,labels=[f'{v:.2f}' for v in means],padding=6);ax.set_ylim(0,max(means)*1.2);ax.set(xticks=x,xticklabels=labels,title=title,ylabel=unit);ax.grid(axis='y',alpha=.25)
stage_cols=['imu_undistort_ms','downsample_ms','map_prepare_ms','estimate_ms','map_update_ms','output_other_ms'];stage_names=['IMU + undistort','Downsample','Map preparation','Correspondence + estimation','Map update','Output + other'];bottom=np.zeros(2)
for col,label in zip(stage_cols,stage_names):
 val=np.array([base[base.algorithm==a][col].mean() for a in algos]);axes[1,0].bar(x,val,bottom=bottom,label=label);bottom+=val
axes[1,0].set(xticks=x,xticklabels=labels,title='Where per-scan time is spent',ylabel='ms / scan');axes[1,0].legend(fontsize=7);axes[1,0].grid(axis='y',alpha=.25)
fig.savefig(OUT/'compute.png');fig.savefig(OUT/'compute.pdf');pdf.savefig(fig);plt.close(fig)
# Raw frame distributions, not just averages.
fig,ax=plt.subplots(figsize=(10,5),constrained_layout=True)
for alg in algos:
 data=[]
 for folder in (OUT/'raw').glob('deployed-'+alg+'-*'):
  if (folder/'run.json').exists():
   s=pd.read_csv(folder/'stages.csv');data.extend(s[(s.stamp>=lo)&(s.stamp<=hi)].total_ms.to_numpy())
 data=np.sort(data);ax.plot(data,np.arange(1,len(data)+1)/len(data),label=NAMES[alg],color=COLORS[alg])
ax.set(xlabel='Processing time [ms / scan]',ylabel='Cumulative fraction',title='Processing latency distribution');ax.legend();ax.grid(alpha=.25);fig.savefig(OUT/'latency_cdf.png');pdf.savefig(fig);plt.close(fig)
(OUT/'evaluation_window.json').write_text(json.dumps(dict(start_sim_s=lo,end_sim_s=hi,path_length_m=float(np.linalg.norm(np.diff(GTp,axis=0),axis=1).sum()),ground_truth_samples=len(gt),alignment='initial stationary interval 5-15s, rigid transform from pose; no whole-path fit or scale',grid_hz=10),indent=2))

# CPU budget includes the high-rate prediction, input handling and ROS executor.
fig,ax=plt.subplots(figsize=(9,5),constrained_layout=True)
core=np.array([base[base.algorithm==a].frame_cpu_ms.mean() for a in algos])
whole=np.array([base[base.algorithm==a].whole_cpu_ms_per_scan.mean() for a in algos])
ax.bar(x,core,label='Profiled scan processing');ax.bar(x,np.maximum(0,whole-core),bottom=core,label='Input / high-rate prediction / ROS / other')
ax.set(xticks=x,xticklabels=labels,ylabel='CPU ms per scan (sum over threads)',title='Whole-process CPU versus scan-core CPU');ax.legend(fontsize=8);ax.grid(axis='y',alpha=.25)
fig.savefig(OUT/'cpu_breakdown.png');pdf.savefig(fig);plt.close(fig)
if (summary.profile=='matched').any():
 fig,axes=plt.subplots(1,3,figsize=(15,5),constrained_layout=True);profiles=['deployed','matched'];xx=np.arange(2)
 for ax,col,title,unit in [(axes[0],'total_ms','Processing time','ms / scan'),(axes[1],'selected_points','Selected points','points / scan'),(axes[2],'imu_ate_rmse_m','IMU odometry position RMSE','cm')]:
  for i,alg in enumerate(algos):
   values=[]
   for profile in profiles:
    data=summary[(summary.profile==profile)&(summary.algorithm==NAMES[alg])]
    values.append(float(data.iloc[0][col])*(100 if col=='imu_ate_rmse_m' else 1) if len(data) else np.nan)
   ax.bar(xx+(i-.5)*.36,values,width=.36,color=COLORS[alg],label=NAMES[alg])
  ax.set(xticks=xx,xticklabels=['Current config','Matched config'],title=title,ylabel=unit);ax.grid(axis='y',alpha=.25)
 axes[0].legend(fontsize=8);fig.suptitle('Configuration control: matched voxel=0.5m, max iterations=4, accel noise=0.1; matched profile is one run',fontsize=10)
 fig.savefig(OUT/'configuration_effect.png');pdf.savefig(fig);plt.close(fig)
fig=plt.figure(figsize=(11.7,8.3));fig.suptitle('Measured comparison: Super-LIO vs FAST-LIVO2 (camera OFF)',fontsize=15,y=.96)
ax=fig.add_axes([.06,.35,.88,.54]);ax.axis('off')
native={a:summary[(summary.profile=='deployed')&(summary.algorithm==NAMES[a])].iloc[0] for a in algos}
items=[('Mean processing [ms]','total_ms',1),('P95 processing [ms]','p95_ms',1),('Process CPU [% of one core]','cpu_percent',1),('Peak RSS [MiB]','peak_rss_mib',1),('Selected points / scan','selected_points',1),('Position RMSE [cm]','imu_ate_rmse_m',100),('Position P95 error [cm]','imu_ate_p95_m',100),('Maximum position error [cm]','imu_ate_max_m',100),('Orientation RMSE [deg]','imu_rotation_rmse_deg',1)]
values=[[label]+[f"{native[a][key]*scale:.2f}" for a in algos] for label,key,scale in items]
table=ax.table(cellText=values,colLabels=['Metric']+labels,loc='center',cellLoc='center',colWidths=[.46,.24,.30]);table.auto_set_font_size(False);table.set_fontsize(10);table.scale(1,1.6)
for c in range(3):table[(0,c)].set_facecolor('#dcebe6')
fig.text(.07,.25,'Current configs: 3 independent restarts per algorithm; same recorded sensor sequence.\n'
 'CPU: 100% is one logical core; mapper affinity and parallelism capped at 4.\n'
 'Accuracy: mounted IMU poses, initial stationary alignment (5-15s) only; no scale or whole-path fitting.\n'
 'Evaluation: 25.01-120.76 simulated seconds, 27.52 m path, common 10 Hz evaluation grid.\n'
 'Secondary matched-config check: voxel=0.5m, iterations=4, accel noise=0.1; 1 run per algorithm.\n'
 'FAST-LIVO2 LIO mode is NOT FAST-LIO2; these results do not reproduce the paper baseline.',fontsize=10,linespacing=1.6)
pdf.savefig(fig);plt.close(fig)
pdf.close()
