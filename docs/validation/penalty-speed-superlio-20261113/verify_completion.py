"""Final verification with the user's explicit completion-first accuracy policy."""
import hashlib,json,shutil,subprocess,time,os
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parent;WS=ROOT.parents[2]
while not(ROOT/'completion-phase-finished.json').exists():time.sleep(1)
while Path(f'/tmp/zav-sim-session-{os.getuid()}.json').exists():time.sleep(1)
time.sleep(5)
def evaluate(name):
 subprocess.run(['python3',str(ROOT/'evaluate.py'),name],check=True)
 return json.loads((ROOT/name/'evaluation.json').read_text())
def persist(attempts,selected=None):
 payload={'selection_policy':'user_completion_clearance_priority_20261113','attempts':attempts,'selected':selected['seed']if selected else None,'runs':selected['runs']if selected else[],'all_passed':bool(selected),'accuracy_is_reported_not_filtered':True}
 (ROOT/'verification-completion.json').write_text(json.dumps(payload,indent=2)+'\n')
fallback={'seed':'balanced_smooth-v65','runs':[],'all_passed':False}
for index in range(1,4):
 r=evaluate(f'verify-balanced_smooth-v65-{index}');r['profile_matches_seed']=yaml.safe_load((ROOT/r['trial']/'loaded-high_speed.yaml').read_text())==yaml.safe_load((ROOT/'balanced_smooth-v65/loaded-high_speed.yaml').read_text());fallback['runs'].append(r)
fallback['all_passed']=all(r['eligible']and r['profile_matches_seed']for r in fallback['runs'])
assert fallback['all_passed'],'Previously completed fallback failed physical checks; inspect before selection'
attempts=[fallback];best=fallback;fallback_time=sum(r['tracking_sim_s']for r in fallback['runs'])/3
names=['completion-balanced-v10','completion-balanced-v9','completion-balanced-v8','completion-balanced-v725','balanced-v75']
candidates=[]
for name in names:
 if(ROOT/name/'evaluation.json').exists():
  r=evaluate(name)
  if r['eligible']:candidates.append(r)
candidates.sort(key=lambda r:r['ranking_time_sim_s'])
for candidate in candidates:
 if candidate['ranking_time_sim_s']>fallback_time*1.05:continue
 seed=candidate['trial'];config=ROOT/seed/'loaded-high_speed.yaml';attempt={'seed':seed,'runs':[],'all_passed':False};attempts.append(attempt)
 for index in range(1,4):
  name=f'final-completion-{seed}-{index}';r=subprocess.run(['python3',str(ROOT/'run_trial.py'),name,'--boundary','{}','--reference-config',str(config)],cwd=WS)
  if r.returncode:raise RuntimeError(name)
  result=evaluate(name);result['profile_matches_seed']=yaml.safe_load((ROOT/name/'loaded-high_speed.yaml').read_text())==yaml.safe_load(config.read_text());attempt['runs'].append(result);persist(attempts)
  time.sleep(4)
  if not result['eligible']or not result['profile_matches_seed']:break
 attempt['all_passed']=len(attempt['runs'])==3 and all(r['eligible']and r['profile_matches_seed']for r in attempt['runs'])
 if attempt['all_passed']:
  mean=sum(r['tracking_sim_s']for r in attempt['runs'])/3
  # Treat <=1% as a practical time tie and prefer higher sustained speed.
  if mean<=fallback_time*1.01:best=attempt;break
 persist(attempts)
recommended=ROOT/'recommended.yaml';recommended.write_text('# GUI penalty tuning: high_speed + super_lio only.\n# See docs/validation/penalty-speed-superlio-20261113/README.md.\n'+(ROOT/best['seed']/'loaded-high_speed.yaml').read_text())
source=WS/'src/zav_super/config/high_speed_super_lio.yaml';expected=json.loads((ROOT/'policy-source-hash.json').read_text())
assert hashlib.sha256(source.read_bytes()).hexdigest()==expected['sha256'],'Source changed while testing; recommendation saved separately'
initial=json.loads((ROOT/'initial-hashes.json').read_text())
for p,h in initial.items():
 if p!='src/zav_super/config/high_speed_super_lio.yaml':assert hashlib.sha256((WS/p).read_bytes()).hexdigest()==h,p
shutil.copy2(recommended,source);shutil.copy2(source,WS/'install/zav_super/share/zav_super/config/high_speed_super_lio.yaml')
persist(attempts,best)
shutil.copy2(ROOT/'verification.json',ROOT/'verification-previous-strict.json')
shutil.copy2(ROOT/'verification-completion.json',ROOT/'verification.json')
print('FINAL COMPLETION-PRIORITY PROFILE APPLIED:',best['seed'],flush=True)
