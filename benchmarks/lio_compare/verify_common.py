#!/usr/bin/env python3
"""Fail if either mapper processes different raw-point cardinalities or frame times."""
from pathlib import Path
import json,csv
import numpy as np
R=Path(__file__).resolve().parents[2]/'reports/lio-common-20261102'
expected=np.array(json.loads((R/'expected_points.json').read_text()))
results=[];counts=[]
for p in sorted((R/'raw').glob('*/run.json')):
 run=json.loads(p.read_text());counts.append(run['input_counts'])
 with (p.parent/'stages.csv').open() as f:
  a=[r for r in csv.DictReader(f) if 51.00999885983765<=float(r['stamp'])<=80.75 and float(r['input_points'])>0]
 t=np.array([float(r['stamp']) for r in a]);n=np.array([int(r['selected_points']) for r in a]);inp=np.array([int(r['input_points']) for r in a])
 assert len(a)==len(expected),(p,len(a),len(expected))
 assert np.max(np.abs(t-expected[:,0]))<1e-5,(p,np.max(np.abs(t-expected[:,0])))
 assert np.array_equal(n,expected[:,1]),(p,'point identity/cardinality mismatch')
 assert np.array_equal(n,inp),(p,'native filtering changed cardinality')
 results.append(dict(run=p.parent.name,frames=len(a),selected_mean=float(n.mean()),max_timestamp_difference_s=float(np.max(np.abs(t-expected[:,0])))))
assert len(results)==6,(len(results),'need 3 runs each')
assert all(x==counts[0] for x in counts)
(R/'point_verification.json').write_text(json.dumps(dict(passed=True,input_counts=counts[0],runs=results),indent=2))
print(json.dumps(results,indent=2))
