"""Vehicle pose and body-frame vectors must use one PhysX tick, without reading USD.
Run the actual update_state method with a fake dynamic-control interface so this
regression test needs neither a GPU nor an Isaac Sim install.
"""
import ast
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation

path=Path(__file__).resolve().parents[1]/'pegasus/simulator/logic/vehicles/vehicle.py'
module=ast.parse(path.read_text());method=next(n for c in module.body if isinstance(c,ast.ClassDef) for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='update_state')
namespace={'np':np,'Rotation':Rotation}
exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),namespace)

def forbidden_usd(*args):
    raise AssertionError('Physics-state update read render-rate USD pose')

rotation=Rotation.from_euler('xyz',[25,-10,70],degrees=True);q=rotation.as_quat();world_v=np.array([1.,2.,3.]);world_w=np.array([.5,-.3,1.2]);world_a=np.array([.2,-.4,.7]);dt=.005
state=SimpleNamespace(linear_velocity=world_v-world_a*dt)
pose=SimpleNamespace(p=[3.,4.,5.],r=SimpleNamespace(x=q[0],y=q[1],z=q[2],w=q[3]))
dc=SimpleNamespace(get_rigid_body=lambda _:1,get_rigid_body_pose=lambda _:pose,get_rigid_body_angular_velocity=lambda _:world_w,get_rigid_body_linear_velocity=lambda _:world_v)
vehicle=SimpleNamespace(_stage_prefix='/test',_state=state,get_dc_interface=lambda:dc,_world=SimpleNamespace(stage=SimpleNamespace(GetPrimAtPath=forbidden_usd)))
namespace['update_state'](vehicle,dt)
np.testing.assert_allclose(state.position,pose.p)
assert (rotation.inv()*Rotation.from_quat(state.attitude)).magnitude()<1e-12
np.testing.assert_allclose(state.angular_velocity,rotation.inv().apply(world_w),atol=1e-12)
np.testing.assert_allclose(state.linear_body_velocity,rotation.inv().apply(world_v),atol=1e-12)
np.testing.assert_allclose(state.linear_acceleration,world_a,atol=1e-10)
print('PASS: pose, gyro frame, body velocity and acceleration use PhysX; no USD access')
