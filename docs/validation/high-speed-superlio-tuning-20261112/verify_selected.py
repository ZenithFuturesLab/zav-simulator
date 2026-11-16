"""Install a scene/backend-only profile and verify it in three fresh GUI sessions."""
import argparse,json,shutil,subprocess,time
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parent;WS=ROOT.parents[2]
p=argparse.ArgumentParser();p.add_argument('trial');p.add_argument('--prefix',required=True);a=p.parse_args()
selected=ROOT/a.trial
source=WS/'src/zav_super/config/high_speed_super_lio.yaml'
source.write_text('# Isaac Sim + RViz GUI tuning, high_speed + super_lio only.\n# See docs/validation/high-speed-superlio-tuning-20261112/README.md.\n'+(selected/'loaded-high_speed.yaml').read_text())
share=WS/'install/zav_super/share/zav_super'
shutil.copy2(source,share/'config'/source.name)
shutil.copy2(WS/'src/zav_super/launch/benchmark.launch.py',share/'launch/benchmark.launch.py')
# Other backends retain the exact installed configuration present before tuning.
shutil.copyfile(ROOT/'v25-a2-installed/loaded-high_speed.yaml',share/'config/high_speed.yaml')
results=[]
for index in range(1,4):
 name=f'{a.prefix}-repeat-{index}'
 r=subprocess.run(['python3',str(ROOT/'run_trial.py'),name,'--boundary','{}','--reference-config',str(source)],cwd=WS)
 if r.returncode:raise RuntimeError(f'Trial process failed: {name}')
 subprocess.run(['python3',str(ROOT/'analyze.py'),str(ROOT/name)],check=True,cwd=WS)
 report=json.loads((ROOT/name/'metrics.json').read_text());report['profile_matches_source']=yaml.safe_load((ROOT/name/'loaded-high_speed.yaml').read_text())==yaml.safe_load(source.read_text());results.append(report)
 (ROOT/f'{a.prefix}-verification.json').write_text(json.dumps({'selected_exploration_trial':a.trial,'runs':results,'all_passed':len(results)==3 and all(r['accepted'] and r['profile_matches_source'] for r in results)},indent=2)+'\n')
 if not report['accepted']:break
 time.sleep(4)
if len(results)!=3 or not all(r['accepted'] and r['profile_matches_source'] for r in results):
 raise SystemExit(1)
