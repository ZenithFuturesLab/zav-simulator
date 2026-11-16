"""Verify fastest eligible exploration profiles in fresh GUI sessions, then apply."""
import hashlib,json,shutil,subprocess,time
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parent;WS=ROOT.parents[2]
while not(ROOT/'bracket-finished.json').exists():time.sleep(1)
while Path(f'/tmp/zav-sim-session-{__import__("os").getuid()}.json').exists():time.sleep(1)
time.sleep(5)
rows=[json.loads(p.read_text())for p in ROOT.glob('*/evaluation.json')]
candidates=sorted([r for r in rows if r['eligible']],key=lambda r:r['ranking_time_sim_s'])
(ROOT/'verification-order.json').write_text(json.dumps([r['trial']for r in candidates],indent=2)+'\n')
attempts=[];selected=None
for candidate in candidates:
 seed=candidate['trial'];config=ROOT/seed/'loaded-high_speed.yaml';attempt={'seed':seed,'runs':[],'all_passed':False};attempts.append(attempt)
 for index in range(1,4):
  name=f'verify-{seed}-{index}'
  r=subprocess.run(['python3',str(ROOT/'run_trial.py'),name,'--boundary','{}','--reference-config',str(config)],cwd=WS)
  if r.returncode:raise RuntimeError(f'Experiment driver failed: {name}')
  subprocess.run(['python3',str(ROOT/'evaluate.py'),name],check=True)
  result=json.loads((ROOT/name/'evaluation.json').read_text());result['profile_matches_seed']=yaml.safe_load((ROOT/name/'loaded-high_speed.yaml').read_text())==yaml.safe_load(config.read_text());attempt['runs'].append(result)
  (ROOT/'verification.json').write_text(json.dumps({'attempts':attempts,'selected':None,'all_passed':False},indent=2)+'\n')
  time.sleep(4)
  if not result['eligible']or not result['profile_matches_seed']:break
 attempt['all_passed']=len(attempt['runs'])==3 and all(r['eligible']and r['profile_matches_seed']for r in attempt['runs'])
 if attempt['all_passed']:selected=attempt;break
if selected is None:raise RuntimeError('No candidate passed three fresh GUI flights; source settings remain unchanged')
recommended=ROOT/'recommended.yaml';recommended.write_text('# GUI penalty tuning: high_speed + super_lio only.\n# See docs/validation/penalty-speed-superlio-20261113/README.md.\n'+(ROOT/selected['seed']/'loaded-high_speed.yaml').read_text())
source=WS/'src/zav_super/config/high_speed_super_lio.yaml'
initial=json.loads((ROOT/'initial-hashes.json').read_text());assert hashlib.sha256(source.read_bytes()).hexdigest()==initial[str(source.relative_to(WS))],'Source was edited during tuning; recommendation retained separately'
# The search only changed planner penalties and three dynamic boundaries.
baseline=yaml.safe_load((ROOT/'initial.yaml').read_text());final=yaml.safe_load(recommended.read_text());other=__import__('copy').deepcopy(final)
other['traj_opt']['boundary']=baseline['traj_opt']['boundary']
for family in ('exp_traj','backup_traj'):
 for key in list(other['traj_opt'][family]):
  if key.startswith('penna_'):other['traj_opt'][family][key]=baseline['traj_opt'][family][key]
assert other==baseline,'Unexpected non-penalty planner changes'
for p,h in initial.items():
 if p!=str(source.relative_to(WS)):assert hashlib.sha256((WS/p).read_bytes()).hexdigest()==h,p
shutil.copy2(recommended,source);shutil.copy2(source,WS/'install/zav_super/share/zav_super/config/high_speed_super_lio.yaml')
(ROOT/'verification.json').write_text(json.dumps({'attempts':attempts,'selected':selected['seed'],'runs':selected['runs'],'all_passed':True,'source_applied':str(source)},indent=2)+'\n')
print('VERIFIED AND APPLIED',selected['seed'],flush=True)
