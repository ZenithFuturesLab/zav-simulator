# [ZAV 변경] RTX 점별 실제 시각을 보존해 FAST-LIVO CustomMsg용 100 ms 패킷을 만듭니다.
"""Group real RTX acquisition timestamps into 100 ms Livox-compatible packets.

Wire format: little-endian uint64 start_ns, uint32 count, then 20-byte records:
float32 x/y/z, uint32 offset_ns, uint8 reflectivity/line/tag/reserved.
The rotary model's emitters are assigned logical Livox lines modulo scan_lines;
this does not reproduce the proprietary Mid360 non-repetitive firing pattern.
"""
import struct
import numpy as np

POINT_DTYPE = np.dtype([
    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("offset", "<u4"),
    ("reflectivity", "u1"), ("line", "u1"), ("tag", "u1"), ("reserved", "u1"),
])


class LivoxPackets:
    def __init__(self, scan_lines=4, period_ns=100_000_000):
        if not 1 <= scan_lines <= 128:
            raise ValueError("Livox scan_lines must be between 1 and 128")
        self.scan_lines = scan_lines
        self.period_ns = period_ns
        self._bucket = None
        self._parts = []
        self._last_timestamp = -1

    def push(self, result, sim_time_ns):
        xyz = np.asarray(result["data"], dtype="<f4").reshape(-1, 3)
        arrays = {}
        for key in ("timestamp", "emitterId", "intensity"):
            if key not in result:
                raise RuntimeError(f"RTX Livox output requires real {key} metadata; got {list(result)}")
            values = np.asarray(result[key]).reshape(-1)
            if len(values) != len(xyz):
                raise RuntimeError(f"RTX {key} length {len(values)} != XYZ length {len(xyz)}")
            arrays[key] = values
        timestamps = arrays["timestamp"].astype(np.int64)
        valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > .01)
        valid &= timestamps > self._last_timestamp
        if not np.any(valid):
            return []
        xyz = xyz[valid]
        timestamps = timestamps[valid]
        # Do not silently relabel wall-clock or unrelated RTX timestamps as sim time.
        if abs(int(timestamps.max()) - int(sim_time_ns)) > 1_000_000_000:
            raise RuntimeError(f"RTX acquisition clock differs from simulation: {timestamps.min()}..{timestamps.max()} vs {sim_time_ns}")
        order = np.argsort(timestamps, kind="stable")
        xyz = xyz[order]
        timestamps = timestamps[order]
        intensity = np.nan_to_num(arrays["intensity"][valid][order], nan=0., posinf=1., neginf=0.)
        emitters = arrays["emitterId"][valid][order]
        points = np.zeros(len(xyz), dtype=POINT_DTYPE)
        for i, axis in enumerate("xyz"):
            points[axis] = xyz[:, i]
        points["reflectivity"] = np.clip(intensity * 255., 0, 255).astype(np.uint8)
        points["line"] = emitters.astype(np.uint32) % self.scan_lines
        points["tag"] = 0x10
        buckets = timestamps // self.period_ns
        packets = []
        for bucket in np.unique(buckets):
            bucket = int(bucket)
            if self._bucket is not None and bucket != self._bucket:
                packet = self._flush()
                if packet is not None:
                    packets.append(packet)
            self._bucket = bucket
            selected = buckets == bucket
            part = points[selected].copy()
            part["offset"] = timestamps[selected] - bucket * self.period_ns
            self._parts.append(part)
        self._last_timestamp = int(timestamps.max())
        return packets

    def _flush(self):
        if not self._parts:
            return None
        points = np.concatenate(self._parts)
        self._parts.clear()
        # Set the header to the first acquired point, with nonnegative offsets.
        first_offset = int(points["offset"][0])
        start_ns = self._bucket * self.period_ns + first_offset
        points["offset"] -= first_offset
        return struct.pack("<QI", start_ns, len(points)) + points.tobytes()
