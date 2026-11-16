"""Ensure floor and unknown contacts cannot silently become obstacle-free data."""
import sys
import unittest
import collections
from types import SimpleNamespace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/super"))
from contact_diagnostics import classify_contact, ContactDiagnostics


class ContactClassification(unittest.TestCase):
    def test_floor_in_either_order(self):
        root = "/World/quadrotor"
        for pair in [(root + "/body/guard", "/World/layout/floor"),
                     ("/World/layout/floor/mesh", root + "/body/guard")]:
            self.assertEqual(classify_contact(*pair, root), "floor")

    def test_obstacle_and_unrecognized_external(self):
        body = "/World/quadrotor/body/guard"
        self.assertEqual(classify_contact(body, "/World/layout/surface_3", "/World/quadrotor"), "obstacle")
        self.assertEqual(classify_contact("/World/unknown_mesh", body, "/World/quadrotor"), "other_external")
        self.assertEqual(classify_contact(body, "/World/layout/floor_post", "/World/quadrotor"), "other_external")

    def test_vehicle_identity_and_self_collision(self):
        root = "/World/quadrotor"
        self.assertIsNone(classify_contact(root + "_other/body", "/World/layout/surface_1", root))
        self.assertEqual(classify_contact(root + "/body", root + "/rotor0", root), "vehicle_self")

    def test_strongest_contact_is_retained_beyond_first_eight(self):
        d = ContactDiagnostics.__new__(ContactDiagnostics)
        d.clock = lambda: 1.
        d.vehicle_root = "/World/quadrotor"
        d.path_cache = {1: d.vehicle_root + "/body", 2: "/World/layout/surface_1"}
        d.types = SimpleNamespace(CONTACT_FOUND=1, CONTACT_LOST=2)
        d.counts = collections.Counter()
        d.events = collections.deque()
        d.callback_histogram = collections.Counter()
        d.callback_calls = d.callback_ns = d.callback_max_ns = 0
        d.errors = d.dropped_events = 0
        d.first_report_sim = d.last_report_sim = None
        h = SimpleNamespace(collider0=1, collider1=2, actor0=1, actor1=2,
                            type=1, num_contact_data=12, contact_data_offset=0)
        points = [SimpleNamespace(impulse=(3., 4., 0.) if i == 10 else (0., 0., 0.),
                                  position=(float(i), 0., 0.), normal=(0., 0., 1.),
                                  separation=-.001 if i == 10 else .01) for i in range(12)]
        d._callback([h], points)
        self.assertEqual(d.errors, 0)
        event = d.events[0]
        self.assertEqual(len(event["points_first8"]), 8)
        self.assertEqual(event["max_impulse_norm"], 5.)
        self.assertEqual(event["strongest_contact"]["position"], [10., 0., 0.])
        self.assertEqual(event["closest_contact"]["separation"], -.001)


if __name__ == "__main__":
    unittest.main()
