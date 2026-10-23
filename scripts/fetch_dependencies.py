#!/usr/bin/env python3
"""Fetch exact upstream revisions; never reset an existing checkout."""
import argparse,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def run(*args): subprocess.run(args,check=True)
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');a=p.parse_args()
 for name,spec in json.loads((ROOT/'dependencies.lock.json').read_text()).items():
  dest=ROOT/'src'/name
  print(name,spec['commit'],flush=True)
  if a.dry_run: continue
  if not dest.exists():
   run('git','clone','--no-checkout',spec['url'],str(dest))
   run('git','-C',str(dest),'checkout','--detach',spec['commit'])
  else:
   actual=subprocess.check_output(['git','-C',str(dest),'rev-parse','HEAD'],text=True).strip()
   if actual!=spec['commit']: raise SystemExit(f'{dest}: revision mismatch; move it aside manually. No reset performed.')
  if spec.get('recursive'):run('git','-C',str(dest),'submodule','update','--init','--recursive','--jobs','2')
  (dest/'COLCON_IGNORE').touch()
if __name__=='__main__': main()
