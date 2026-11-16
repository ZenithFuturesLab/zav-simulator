import copy,json
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parent
base=yaml.safe_load((ROOT/'initial.yaml').read_text())
profiles={
 'original':{},
 'relaxed_jerk':{'exp_traj':{'penna_jerk':5e10,'penna_attract':1e6},'backup_traj':{'penna_jerk':1e9}},
 'smooth_constraints':{'exp_traj':{'penna_jerk':5e12,'penna_acc':2e9,'penna_vel':1e9,'penna_attract':1e6},'backup_traj':{'penna_jerk':1e10,'penna_acc':5e8,'penna_vel':5e8}},
 'time_pressure':{'exp_traj':{'penna_t':3e8,'penna_vel':1e9,'penna_acc':1e9,'penna_attract':1e6},'backup_traj':{'penna_t':3e6,'penna_ts':3e7,'penna_acc':3e8,'penna_vel':3e8,'penna_jerk':1e9}},
 'balanced_smooth':{'exp_traj':{'penna_t':3e7,'penna_jerk':1e10,'penna_attract':1e5,'penna_pos':1e9},'backup_traj':{'penna_t':3e5,'penna_ts':3e6,'penna_jerk':1e10,'penna_acc':5e8,'penna_vel':5e8}}
}
(ROOT/'configs').mkdir(exist_ok=True)
for name,updates in profiles.items():
 d=copy.deepcopy(base)
 for family,values in updates.items():d['traj_opt'][family].update(values)
 d['traj_opt']['boundary'].update(max_vel=6.5,max_acc=2.5,max_jerk=10.)
 (ROOT/'configs'/f'{name}-v65.yaml').write_text(yaml.safe_dump(d,sort_keys=False))
(ROOT/'profiles.json').write_text(json.dumps(profiles,indent=2)+'\n')
print('Prepared five penalty profiles at shared boundary 6.5 / 2.5 / 10 / 1.2')
