"""GT must read PhysX directly even when an estimator/state cache is wrong."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from scipy.spatial.transform import Rotation

path = Path(__file__).resolve().parents[1] / "zav_sim.py"
tree = ast.parse(path.read_text())
method = next(n for c in tree.body if isinstance(c, ast.ClassDef)
              for n in c.body if isinstance(n, ast.FunctionDef) and n.name == "publish_ground_truth")
namespace = {"np": np, "Rotation": Rotation}
exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), namespace)

rotation = Rotation.from_euler("xyz", [10, -5, 30], degrees=True)
q = rotation.as_quat()
position = np.array([1., 2., 3.])
velocity = np.array([.2, -.4, .1])
omega = np.array([.3, -.2, .4])
offset = np.array([.04, .02, .15])
mount = Rotation.from_euler("y", 22, degrees=True)
dc = NS(get_rigid_body=lambda _: 1,
        get_rigid_body_pose=lambda _: NS(p=position, r=NS(**dict(zip("xyzw", q)))),
        get_rigid_body_angular_velocity=lambda _: omega,
        get_rigid_body_linear_velocity=lambda _: velocity)

class Drone:
    _stage_prefix = "/drone"
    def get_dc_interface(self): return dc
    @property
    def state(self): raise AssertionError("GT read cached/estimated state")

def message():
    return NS(header=NS(stamp=NS()), pose=NS(pose=NS(position=NS(), orientation=NS())),
              twist=NS(twist=NS(linear=NS(), angular=NS())))

published = []
app = NS(drone=Drone(), world=NS(current_time=1.25), _imu_offset=offset,
         _imu_rotation=mount, _truth_type=message, _truth_pub=NS(publish=published.append))
namespace["publish_ground_truth"](app)
msg = published[0]
np.testing.assert_allclose([getattr(msg.pose.pose.position, k) for k in "xyz"],
                           position + rotation.apply(offset))
actual = Rotation.from_quat([getattr(msg.pose.pose.orientation, k) for k in "xyzw"])
assert ((rotation * mount).inv() * actual).magnitude() < 1e-12
np.testing.assert_allclose([getattr(msg.twist.twist.linear, k) for k in "xyz"],
                           (rotation * mount).inv().apply(velocity + np.cross(omega, rotation.apply(offset))))
assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (1, 250_000_000)
print("PASS: independent PhysX GT pose, sensor lever arm, twist and measurement timestamp")
