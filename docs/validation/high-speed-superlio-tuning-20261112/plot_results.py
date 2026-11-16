import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent;WS=ROOT.parents[2]
sys.path.insert(0,str(WS/'simulator/pegasus/simulator/logic/sensors'))
from fastlivo_calibration import load_calibration
cal=load_calibration(WS/'src/FAST-LIVO2/config',[.01215,0,.11503],[0,22,0]);mount=Rotation.from_euler('ZYX',cal['imu_orientation'],degrees=True)
fig,axes=plt.subplots(1,3,figsize=(15,5))
for name in sys.argv[1:]:
 folder=ROOT/name;events=[json.loads(l) for l in (folder/'odom-events.jsonl').read_text().splitlines()];truth={r['stamp']:r for r in events if r['key']=='truth_imu'};tt=np.array(sorted(truth));tr=[truth[t]for t in tt]
 rot=Rotation.from_quat(np.array([r['q']for r in tr])[:,[1,2,3,0]])*mount.inv();body=np.array([r['p']for r in tr])-rot.apply(cal['imu_position'])
 lio={r['stamp']:r for r in events if r['key']=='lio_body' and tt[0]<=r['stamp']<=tt[-1]};lt=np.array(sorted(lio));lp=np.array([lio[t]['p']for t in lt]);actual=np.column_stack([np.interp(lt,tt,body[:,i])for i in range(3)]);err=np.linalg.norm(lp-actual,axis=1)
 axes[0].plot(body[:,0],body[:,1],label=name,lw=1);axes[1].plot(lt-lt[0],err,label=name,lw=1)
 dt=np.diff(tt);speed=np.linalg.norm(np.diff(body,axis=0)/dt[:,None],axis=1);axes[2].plot(tt[1:]-tt[0],speed,label=name,lw=1)
axes[0].set(xlabel='Physical body X (m)',ylabel='Physical body Y (m)',title='Physical round trip');axes[0].axis('equal')
axes[1].set(xlabel='Simulation time (s)',ylabel='LIO position error (m)',title='Timestamp-aligned LIO error');axes[1].axhline(.5,c='red',ls='--',lw=1)
axes[2].set(xlabel='Simulation time (s)',ylabel='Physical speed (m/s)',title='Physical body speed')
for ax in axes:ax.grid(True,alpha=.3);ax.legend(fontsize=7)
fig.tight_layout();fig.savefig(ROOT/'comparison.png',dpi=170)
