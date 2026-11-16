"""Validate density contrast, nontrivial passage and valid original mission pads."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/super'))
from pcd_scene import read_pcd
from mixed_scene import ensure_mixed_map

d=Path(__file__).resolve().parents[1]/'assets/super';p=ensure_mixed_map(d);recipe=json.loads((d/'mixed_recipe.json').read_text());assert hashlib.sha256(p.read_bytes()).hexdigest()==recipe['generated_pcd_sha256'];cloud=read_pcd(p);obs=cloud[abs(cloud[:,2])>=1e-7];assert np.isfinite(cloud).all();assert len(cloud)==recipe['points']
voxel_lower=np.floor(obs[:,:2].astype(float)/.05)*.05
pad_clearance=[]
for y in [-50.,50.]:
 delta=np.maximum(np.maximum(voxel_lower-[0.,y],[0.,y]-(voxel_lower+.05)),0)
 distance=float(np.linalg.norm(delta,axis=1).min());assert distance>=2.5
 pad_clearance.append(distance)
# Occupied XY area per m² compares density without counting a tall tree twice.
xy=np.unique(np.floor(obs[:,:2]/.2).astype(int),axis=0)*.2+.1
fraction={z['name']:float(np.sum((xy[:,1]>=z['y_min'])&(xy[:,1]<z['y_max']))*.04/(15*(z['y_max']-z['y_min'])))for z in recipe['zones']}
assert fraction['dense_center']>3*max(fraction['open_south'],fraction['open_north']),fraction
# No artificial clear center lane through the dense zone at flight height.
assert np.sum((abs(obs[:,0])<.3)&(abs(obs[:,1])<18)&(abs(obs[:,2]-1.5)<.15))>10
for g in recipe['groups']:
 if g['source']=='dense':assert -18<=g['bounds_min'][1]<=g['bounds_max'][1]<=18
 else:assert g['bounds_max'][1]<=-22 or g['bounds_min'][1]>=22
print(json.dumps({'valid':True,'points':len(cloud),'obstacle_points':len(obs),'pad_xy_voxel_clearance_m':pad_clearance,'xy_occupied_fraction':fraction,'map_sha256':recipe['generated_pcd_sha256']},indent=2))
