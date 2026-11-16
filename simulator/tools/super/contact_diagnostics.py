"""Opt-in, buffered PhysX contact diagnostics; never a flight-control input.

Floor reports are counted without serializing contact points. External contacts
retain their actual collider paths and PhysX contact data. Output is buffered
outside the physics callback, with cumulative coverage/overhead records.
"""
import collections
import json
import math
from pathlib import Path
import time


def under(path, root):
    return path == root or path.startswith(root + "/")


def classify_contact(collider0, collider1, vehicle_root):
    a, b = under(collider0, vehicle_root), under(collider1, vehicle_root)
    if not (a or b):
        return None
    if a and b:
        return "vehicle_self"
    other = collider1 if a else collider0
    if under(other, "/World/layout/floor"):
        return "floor"
    if other.startswith("/World/layout/surface_"):
        return "obstacle"
    return "other_external"


class ContactDiagnostics:
    def __init__(self, stage, vehicle_root, clock, path):
        from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics, PhysicsSchemaTools
        from omni.physx import get_physx_simulation_interface
        from omni.physx.bindings._physx import ContactEventType

        self.clock = clock
        self.vehicle_root = vehicle_root
        self.path = Path(path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("x", buffering=65536)
        self.events = collections.deque()
        self.counts = collections.Counter()
        self.path_cache = {}
        self.decode = PhysicsSchemaTools.intToSdfPath
        self.types = ContactEventType
        self.callback_calls = 0
        self.callback_ns = 0
        self.callback_max_ns = 0
        self.callback_histogram = collections.Counter()
        self.flush_ns = 0
        self.flush_max_ns = 0
        self.last_summary = 0.
        self.first_report_sim = None
        self.last_report_sim = None
        self.first_floor_sim = None
        self.last_floor_sim = None
        self.errors = 0
        self.dropped_events = 0
        bodies = []
        vehicle = stage.GetPrimAtPath(vehicle_root)
        if not vehicle:
            raise RuntimeError(f"Contact diagnostic vehicle missing: {vehicle_root}")
        for prim in Usd.PrimRange(vehicle):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                report = PhysxSchema.PhysxContactReportAPI.Apply(prim)
                report.CreateThresholdAttr().Set(0.)
                bodies.append(str(prim.GetPath()))
        if not bodies:
            raise RuntimeError("Contact diagnostic found no vehicle rigid bodies")
        self._write({"kind": "metadata", "vehicle_root": vehicle_root,
                     "rigid_bodies": bodies, "threshold": 0.,
                     "stage_meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
                     "clock": "world.current_time and host monotonic; frame clock may lag within render interval",
                     "impulse_units": "PhysX native scene units", "visualization": False})
        self.file.flush()
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(self._callback)

    def _write(self, event):
        self.file.write(json.dumps(event, allow_nan=False) + "\n")

    def _path(self, identifier):
        identifier = int(identifier)
        if identifier not in self.path_cache:
            self.path_cache[identifier] = str(self.decode(identifier))
        return self.path_cache[identifier]

    def _queue(self, event):
        if len(self.events) >= 4096:
            self.dropped_events += 1
        else:
            self.events.append(event)

    def _callback(self, headers, data):
        begin = time.perf_counter_ns()
        try:
            stamp = float(self.clock())
            for h in headers:
                c0, c1 = self._path(h.collider0), self._path(h.collider1)
                category = classify_contact(c0, c1, self.vehicle_root)
                if category is None:
                    continue
                event = ("found" if h.type == self.types.CONTACT_FOUND else
                         "lost" if h.type == self.types.CONTACT_LOST else "persists")
                self.counts[category + "_" + event] += 1
                self.counts[category + "_points"] += int(h.num_contact_data)
                if self.first_report_sim is None:
                    self.first_report_sim = stamp
                self.last_report_sim = stamp
                if category == "floor":
                    if self.first_floor_sim is None:
                        self.first_floor_sim = stamp
                    self.last_floor_sim = stamp
                if category in ("floor", "vehicle_self"):
                    continue
                points = []
                min_separation, max_impulse = None, 0.
                strongest_contact = None
                closest_contact = None
                for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                    item = data[i]
                    impulse = [float(item.impulse[j]) for j in range(3)]
                    separation = float(item.separation)
                    norm = math.sqrt(sum(v*v for v in impulse))
                    is_closest = min_separation is None or separation < min_separation
                    is_strongest = strongest_contact is None or norm > max_impulse
                    if len(points) < 8 or is_closest or is_strongest:
                        point = {"position": [float(item.position[j]) for j in range(3)],
                                 "normal": [float(item.normal[j]) for j in range(3)],
                                 "impulse": impulse, "separation": separation}
                        if len(points) < 8:
                            points.append(point)
                        if is_closest:
                            closest_contact = point
                        if is_strongest:
                            strongest_contact = point
                    min_separation = separation if min_separation is None else min(min_separation, separation)
                    max_impulse = max(max_impulse, norm)
                self._queue({"kind": "contact", "sim_time_s": stamp,
                             "wall_monotonic_s": time.monotonic(), "category": category,
                             "event": event, "raw_event_type": str(h.type),
                             "collider0": c0, "collider1": c1,
                             "actor0": self._path(h.actor0), "actor1": self._path(h.actor1),
                             "num_points": int(h.num_contact_data), "min_separation": min_separation,
                             "max_impulse_norm": max_impulse, "points_first8": points,
                             "strongest_contact": strongest_contact, "closest_contact": closest_contact})
        except Exception as exc:
            self.errors += 1
            if self.errors <= 5:
                self._queue({"kind": "callback_error", "message": repr(exc)})
        finally:
            cost = time.perf_counter_ns() - begin
            self.callback_calls += 1
            self.callback_ns += cost
            self.callback_max_ns = max(self.callback_max_ns, cost)
            # Bounded histogram, so no per-tick timing samples grow in memory.
            bucket_us = next((b for b in (10, 25, 50, 100, 250, 500, 1000, 5000)
                              if cost <= b*1000), 1000000)
            self.callback_histogram[bucket_us] += 1

    def summary(self, closed=False):
        return {"kind": "summary", "sim_time_s": float(self.clock()),
                "wall_monotonic_s": time.monotonic(), "closed": closed,
                "counts": dict(self.counts), "callback_errors": self.errors,
                "dropped_events": self.dropped_events,
                "first_report_sim_s": self.first_report_sim, "last_report_sim_s": self.last_report_sim,
                "first_floor_sim_s": self.first_floor_sim, "last_floor_sim_s": self.last_floor_sim,
                "callback_calls": self.callback_calls, "callback_total_ms": self.callback_ns/1e6,
                "callback_max_ms": self.callback_max_ns/1e6,
                "callback_histogram_upper_us": dict(self.callback_histogram),
                "flush_total_ms": self.flush_ns/1e6, "flush_max_ms": self.flush_max_ns/1e6}

    def flush(self, force=False, closed=False):
        now = time.monotonic()
        due = force or now - self.last_summary >= 1.
        if not due and not self.events:
            return
        begin = time.perf_counter_ns()
        while self.events:
            self._write(self.events.popleft())
        if due:
            self._write(self.summary(closed))
            self.last_summary = now
        self.file.flush()
        cost = time.perf_counter_ns() - begin
        self.flush_ns += cost
        self.flush_max_ns = max(self.flush_max_ns, cost)

    def close(self):
        if self.file.closed:
            return
        self.subscription = None
        self.flush(force=True, closed=True)
        self.file.close()
