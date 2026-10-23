#!/usr/bin/env python3
"""Read-only installation checks. Does not launch a simulator or arm a vehicle."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
errors=[]
def check(name,ok):
    print(('OK   ' if ok else 'FAIL ')+name)
    if not ok: errors.append(name)
for path in ['.zav.env','install/setup.bash','src/PX4-Autopilot/build/px4_sitl_default/bin/px4','.local/bin/MicroXRCEAgent','src/PegasusSimulator/examples/zav_sim.py']:
    check(path,(ROOT/path).exists())
for name,spec in json.loads((ROOT/'dependencies.lock.json').read_text()).items():
    p=ROOT/'src'/name
    result=subprocess.run(['git','-C',str(p),'rev-parse','HEAD'],capture_output=True,text=True)
    check(name+' pinned revision',result.returncode==0 and result.stdout.strip()==spec['commit'])
for exe in ['tmux','colcon','cmake','make']:
    check(exe,shutil.which(exe) is not None)
if (ROOT/'install/setup.bash').exists():
    result=subprocess.run(['bash','-c','source "$1/simulation/ros_setup.sh" && ros2 pkg executables zav_control','check',str(ROOT)],capture_output=True,text=True)
    check('ROS flight_manager + lio_px4_bridge',result.returncode==0 and 'flight_manager' in result.stdout and 'lio_px4_bridge' in result.stdout)
    result=subprocess.run(['bash','-c','source "$1/simulation/ros_setup.sh" && ros2 pkg executables fast_livo','check',str(ROOT)],capture_output=True,text=True)
    check('ROS fastlivo_mapping',result.returncode==0 and 'fastlivo_mapping' in result.stdout)
    probe='from scipy.spatial.transform import Rotation; import cv_bridge; from livox_ros_driver2.msg import CustomMsg; from px4_msgs.msg import VehicleStatus; from rclpy.serialization import serialize_message, deserialize_message; [(deserialize_message(serialize_message(cls()), cls)) for cls in (CustomMsg, VehicleStatus)]'
    result=subprocess.run(['bash','-c','source "$1/simulation/ros_setup.sh" && python3 -c "$2"','check',str(ROOT),probe],capture_output=True,text=True)
    check('ROS sensor Python ABI + message serialization',result.returncode==0)
    if result.returncode: print(result.stderr.strip())
print('GPU rendering, sensor streams and flight response require the runtime checks documented in docs/verification.md.')
sys.exit(bool(errors))
