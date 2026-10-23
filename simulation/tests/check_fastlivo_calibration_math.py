"""Check calibration directions and binary point timing without Isaac/ROS."""
import importlib.util
from pathlib import Path
import struct
import numpy as np
from scipy.spatial.transform import Rotation

root = Path(__file__).resolve().parents[1]
logic = root / "pegasus/overlay/logic"
def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
cal = load("cal", logic / "sensors/fastlivo_calibration.py")
packet = load("packets", logic / "backends/livox_packets.py")
config_dir = root.parent / "src/FAST-LIVO2/config"
cfg = cal.load_calibration(config_dir, [.01215, 0, .11503], [0, 22, 0])
r_bl = Rotation.from_euler("Y", 22, degrees=True).as_matrix()
r_bi = Rotation.from_euler("ZYX", cfg["imu_orientation"], degrees=True).as_matrix()
np.testing.assert_allclose(r_bi.T @ r_bl, np.eye(3), atol=1e-8)
np.testing.assert_allclose(r_bi.T @ (np.array([.01215, 0, .11503]) - cfg["imu_position"]), [-.011, -.02329, -.04412], atol=1e-8)
assert cfg["resolution"] == (640, 512)

formatter = packet.LivoxPackets()
result = {"data": np.ones((8, 3), dtype=np.float32),
          "timestamp": np.array([10, 20, 30, 40, 110, 120, 130, 140], dtype=np.int64) * 1_000_000,
          "emitterId": np.arange(8), "intensity": np.ones(8) * .5}
payloads = formatter.push(result, 150_000_000)
assert len(payloads) == 1
start, count = struct.unpack_from("<QI", payloads[0])
points = np.frombuffer(payloads[0], dtype=packet.POINT_DTYPE, offset=12)
assert (start, count) == (10_000_000, 4)
np.testing.assert_array_equal(points["offset"], [0, 10_000_000, 20_000_000, 30_000_000])
np.testing.assert_array_equal(points["line"], [0, 1, 2, 3])
assert formatter.push(result, 150_000_000) == []  # Repeated render result is ignored.
try:
    packet.LivoxPackets().push({**result, "timestamp": np.arange(2)}, 150_000_000)
except RuntimeError:
    pass
else:
    raise AssertionError("Mismatched timing metadata must never be fabricated")
print("PASS: calibrated mount direction, IMU offset, camera size, packet timing, lines, duplicate rejection")
