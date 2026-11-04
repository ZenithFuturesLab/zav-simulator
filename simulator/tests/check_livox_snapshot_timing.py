"""Check frame-time semantics and packetization without Isaac Sim or ROS."""
import importlib.util
from pathlib import Path
import struct
import numpy as np

path = Path(__file__).resolve().parents[1] / "pegasus/simulator/logic/backends/livox_packets.py"
spec = importlib.util.spec_from_file_location("livox_packets", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

packetizer = module.LivoxPackets()
packets = []
for index in range(3):
    start = 10_000_000 + index * 50_000_000
    end = start + 50_000_000
    times = module.snapshot_timestamps(3, start + 2_000_000_000, end + 2_000_000_000,
                                       -2_000_000_000, end)
    np.testing.assert_array_equal(times, [start] * 3)
    result = {"data": np.array([[1., 0., 0.], [2., 0., 0.], [3., 0., 0.]]),
              "timestamp": times, "emitterId": np.arange(3), "intensity": np.ones(3)}
    packets.extend(packetizer.push(result, end))
    assert packetizer.push(result, end) == []  # repeated render read, no duplicate points
assert len(packets) == 1
stamp, count = struct.unpack_from("<QI", packets[0])
points = np.frombuffer(packets[0], dtype=module.POINT_DTYPE, offset=12)
assert stamp == 10_000_000 and count == 6
np.testing.assert_array_equal(points["offset"], [0, 0, 0, 50_000_000, 50_000_000, 50_000_000])
np.testing.assert_allclose(points["x"], [1, 2, 3, 1, 2, 3])
try:
    module.snapshot_timestamps(1, 100, 200, 0, 200_000_000)
except ValueError:
    pass
else:
    raise AssertionError("Unsynchronized sensor clock accepted")
print("PASS: snapshot pose times, preserved XYZ, 100 ms packets, duplicate reads, clock validation")
