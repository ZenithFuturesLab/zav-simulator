"""Pure numerical checks; can run without Isaac Sim or a ROS installation."""
import importlib.util
from pathlib import Path
import sys
import types

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
LOGIC = ROOT / "pegasus/simulator/logic"
# Load real sensor/state modules without importing the Isaac extension package.
for name in ("pegasus", "pegasus.simulator", "pegasus.simulator.logic", "pegasus.simulator.logic.sensors"):
    module = types.ModuleType(name)
    module.__path__ = []
    sys.modules[name] = module

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module

load("pegasus.simulator.logic.rotations", LOGIC / "rotations.py")
State = load("pegasus.simulator.logic.state", LOGIC / "state.py").State
load("pegasus.simulator.logic.sensors.sensor", LOGIC / "sensors/sensor.py")
MountedIMU = load("pegasus.simulator.logic.sensors.mounted_imu", ROOT / "pegasus/simulator/logic/sensors/mounted_imu.py").MountedIMU

config = {"position": [0.01215, 0, 0.11503], "orientation": [0, 22, 0], "update_rate": 250}
sensor = MountedIMU(config)
state = State()
result = sensor.update(state, .004)
expected = Rotation.from_euler("Y", -22, degrees=True).apply([0, 0, 9.80665])
np.testing.assert_allclose(result["linear_acceleration"], expected, atol=1e-10)
np.testing.assert_allclose(result["angular_velocity"], 0, atol=1e-10)

# Verify world attitude, tangential and centripetal acceleration together.
sensor.start()
state.attitude = Rotation.from_euler("XYZ", [15, -10, 70], degrees=True).as_quat()
state.linear_acceleration = np.array([1., 2., 3.])
state.angular_velocity = np.array([0., 0., 2.])
sensor.update(state, .004)
state.angular_velocity = np.array([0., 0., 2.004])
result = sensor.update(state, .004)
r = np.array(config["position"])
specific = Rotation.from_quat(state.attitude).inv().apply(state.linear_acceleration + [0, 0, 9.80665])
specific += np.cross([0, 0, 1], r) + np.cross(state.angular_velocity, np.cross(state.angular_velocity, r))
np.testing.assert_allclose(result["linear_acceleration"], Rotation.from_euler("Y", -22, degrees=True).apply(specific), atol=1e-10)

# 200 Hz configured against a 250 Hz physics clock: no render-rate dependency.
sensor = MountedIMU({**config, "update_rate": 200})
assert sum(sensor.update(state, .004) is not None for _ in range(250)) == 200
sensor.reset()
assert sensor.state is None
print("PASS: gravity, rotated mount/world, angular velocity, lever arm, scheduling, reset")

# The 100 Hz simulator profile must produce one mounted IMU sample per tick.
sensor = MountedIMU({**config, "update_rate": 100})
samples = [sensor.update(state, .01) for _ in range(100)]
assert all(sample is not None for sample in samples)
assert len(samples) == 100
print("PASS: 100 Hz physics produces 100 Hz mounted IMU without render dependence")

# Real PhysX float32 dt must not skip the first 100 Hz sample or leave a
# nearly-full-period residual that doubles the covariance integration time.
dt = float(np.float32(.01))
sensor = MountedIMU({**config, "update_rate": 100, "gyro_noise_density": .01})
samples = [sensor.update(state, dt) for _ in range(100)]
assert all(sample is not None for sample in samples)
np.testing.assert_allclose([sample["gyro_variance"] for sample in samples], .01**2/dt)
print("PASS: float32 10 ms sampling and per-sample covariance")
