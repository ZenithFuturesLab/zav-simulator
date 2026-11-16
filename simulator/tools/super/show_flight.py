#!/usr/bin/env python3
"""Open the latest SUPER flight report, optionally recomputing derivatives."""
import argparse
from pathlib import Path
import subprocess
import sys
from flight_metrics import analyze

p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',nargs='?',type=Path);p.add_argument('--window',type=float,help='Recompute with this smoothing window (seconds)');p.add_argument('--no-open',action='store_true');a=p.parse_args()
root=Path(__file__).resolve().parents[3]/'log/super_flights'
if a.directory:directory=a.directory.resolve()
else:
    candidates=[f.parent for f in root.glob('*/samples.jsonl')if f.stat().st_size]
    if not candidates:p.exit(1,'No flight data yet. Run ./run_super.sh ... --execute first.\n')
    directory=max(candidates,key=lambda f:(f/'samples.jsonl').stat().st_mtime)
if a.window is not None and (a.window<=0 or not __import__('math').isfinite(a.window)):p.error('--window must be positive and finite')
try:
    raw=directory/'samples.jsonl';report=directory/'report.html'
    if a.window is not None or not report.exists() or report.stat().st_mtime<raw.stat().st_mtime:analyze(directory,a.window or .21)
except (OSError,ValueError)as exc:p.exit(1,str(exc)+'\n')
print(directory/'report.html',flush=True)
if not a.no_open:subprocess.run(['xdg-open',str(directory/'report.html')],check=False,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
