#!/usr/bin/env python3
"""Portable packaging, syntax and profile checks; no ROS or GPU required."""
import ast
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
paths=[ROOT/'scripts',ROOT/'simulator',*(p for p in (ROOT/'src').iterdir() if p.name not in {*json.loads((ROOT/'dependencies.lock.json').read_text()), 'PegasusSimulator'})]
for directory in paths:
    for p in directory.rglob('*'):
        if not p.is_file() or any(x in p.parts for x in ['__pycache__','.git','build','install']): continue
        if p.suffix=='.py': ast.parse(p.read_text(),filename=str(p))
        if p.suffix=='.sh': subprocess.run(['bash','-n',str(p)],check=True)
        if p.name=='package.xml': ET.parse(p)
        if p.suffix in ('.py','.sh','.yaml','.xml','.md','.params','.rviz'):
            assert ('/home/'+'mscl') not in p.read_text(),f'Private host path: {p}'
required_packages={
    'FAST-LIVO2':'fast_livo',
    'livox_ros_driver2':'livox_ros_driver2',
    'px4_msgs':'px4_msgs',
    'sophus':'sophus',
    'rpg_vikit/vikit_common':'vikit_common',
    'rpg_vikit/vikit_ros':'vikit_ros',
    'zav_control':'zav_control',
}
for directory, name in required_packages.items():
    manifest=ROOT/'src'/directory/'package.xml'
    assert manifest.is_file(),f'Missing required ROS manifest: {manifest}'
    assert ET.parse(manifest).getroot().findtext('name')==name,manifest
airframe=(ROOT/'simulator/px4/10020_isaac_zav').read_text()
values={}
for line in airframe.splitlines():
    if line.startswith('param set-default '):
        _,_,name,value=line.split();values[name]=float(value)
for name,value in {'MPC_XY_VEL_MAX':12,'MPC_VEL_MANUAL':12,'MPC_XY_CRUISE':1.5,'MPC_MAN_Y_MAX':30,'EKF2_MAG_TYPE':5,'EKF2_GPS_CTRL':0,'EKF2_EV_CTRL':15}.items(): assert values[name]==value,(name,values[name])
for line in (ROOT/'simulator/current_ekf_mpc.params').read_text().splitlines():
    key,value=line.split();assert values[key]==float(value),key
subprocess.run(['bash',str(ROOT/'scripts/bootstrap.sh'),'--dry-run'],check=True,stdout=subprocess.DEVNULL)
print('RELEASE_OK: source syntax, package XML, portable paths, effective profile, dry-run')
