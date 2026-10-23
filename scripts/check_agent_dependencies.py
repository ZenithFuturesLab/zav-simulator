#!/usr/bin/env python3
"""Verify all built DDS source revisions, including Micro-CDR's upstream tag."""
import json
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
paths={
 'fastcdr':'fastcdr/src/fastcdr',
 'fastdds':'fastdds/src/fastdds',
 'microxrcedds_client':'microxrcedds_client/src/microxrcedds_client',
 'microcdr':'microxrcedds_client/src/microxrcedds_client-build/microcdr/src/microcdr',
}
for name,sha in json.loads((ROOT/'agent-dependencies.lock.json').read_text()).items():
    p=ROOT/'build/agent'/paths[name]
    actual=subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip()
    if actual!=sha:raise SystemExit(f'{name}: revision mismatch ({actual}); refusing installation')
    print('PIN_OK',name,sha)
