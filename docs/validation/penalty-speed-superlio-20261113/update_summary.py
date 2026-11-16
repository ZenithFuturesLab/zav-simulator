import json
from pathlib import Path
import yaml
root=Path(__file__).resolve().parent
reports=[]
for p in sorted(root.glob('*/evaluation.json'),key=lambda p:(p.parent/'result.json').stat().st_mtime):
 r=json.loads(p.read_text());reports.append(r)
(root/'summary.json').write_text(json.dumps(reports,indent=2)+'\n')
lines=['# high_speed / Super-LIO penalty tuning — 2026-11-13','','GUI Isaac Sim + RViz only. Repeated verification is required before final adoption. Failed or partially landed flights are excluded from time ranking.','','| Trial | v/a/j caps | Healthy complete | Time sim s (eligible only) | Actual peak / p95 m/s | Peak/cap | LIO max m | Clearance lower bound m |','|---|---|---|---:|---|---:|---:|---:|']
for r in reports:
 b=r['boundary'];time=f"{r['ranking_time_sim_s']:.2f}"if r.get('eligible')else'—'
 lines.append(f"| [{r['trial']}]({r['trial']}/evaluation.json) | {b['max_vel']}/{b['max_acc']}/{b['max_jerk']} | {r.get('healthy_complete')} | {time} | {r.get('actual_peak_mps',0):.3f} / {r.get('actual_p95_mps',0):.3f} | {r.get('peak_speed_attainment',0):.3f} | {r.get('flight_lio_max_m',0):.3f} | {r.get('conservative_body_clearance_m',0):.3f} |")
(root/'progress.md').write_text('\n'.join(lines)+'\n')
print('\n'.join(lines))
