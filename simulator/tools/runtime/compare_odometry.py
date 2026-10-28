#!/usr/bin/env python3
"""Compare FAST-LIVO mounted-IMU pose against timestamp-interpolated Isaac truth.

One fixed SE(3) alignment at the first paired pose removes initial origin/heading.
It is never refitted, so subsequent position and attitude drift remain visible.
"""
import argparse
import bisect
import csv
import math
import time
from collections import deque
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return np.array([p.x, p.y, p.z]), Rotation.from_quat([q.x, q.y, q.z, q.w])


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def interpolate(a, b, t):
    ta, pa, ra = a
    tb, pb, rb = b
    if tb == ta:
        return pa, ra
    f = (t - ta) / (tb - ta)
    return pa + f * (pb - pa), Slerp([ta, tb], Rotation.concatenate([ra, rb]))([t])[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--estimate', default='/aft_mapped_to_init')
    parser.add_argument('--truth', default='/drone0/ground_truth/odometry')
    parser.add_argument('--csv', default='/tmp/zav_odometry_error.csv')
    parser.add_argument('--plot', action='store_true', help='Show live position/attitude error plots')
    args = parser.parse_args()
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.node import Node

    class Compare(Node):
        def __init__(self):
            super().__init__('zav_odometry_comparison')
            self.truth = deque(maxlen=30000)
            self.pending = deque(maxlen=2000)
            self.alignment = None
            self.n = 0
            self.squared = 0.
            self.angle_squared = 0.
            self.latest = None
            self.start = None
            self.last_estimate = None
            self.dropped = 0
            self.rows = deque(maxlen=20000)
            self.file = Path(args.csv).open('w', newline='')
            self.writer = csv.writer(self.file)
            self.writer.writerow(['segment', 'sim_time_s', 'elapsed_s', 'dx_m', 'dy_m', 'dz_m',
                                  'position_error_m', 'attitude_error_deg', 'position_rmse_m',
                                  'attitude_rmse_deg'])
            self.segment = 0
            self.create_subscription(Odometry, args.truth, self.on_truth, qos_profile_sensor_data)
            self.create_subscription(Odometry, args.estimate, self.on_estimate, qos_profile_sensor_data)
            self.create_timer(1., self.report)
            if args.plot:
                import matplotlib.pyplot as plt
                self.plt = plt
                plt.ion()
                self.fig, self.axes = plt.subplots(2, 1, sharex=True)
                self.lines = [ax.plot([], [])[0] for ax in self.axes]
                self.axes[0].set_ylabel('Position error [m]')
                self.axes[1].set_ylabel('Attitude error [deg]')
                self.axes[1].set_xlabel('Simulation elapsed [s]')
                plt.show(block=False)
            print(f'Waiting for {args.estimate} and {args.truth}\nCSV: {args.csv}', flush=True)

        def reset(self):
            self.segment += 1
            self.truth.clear()
            self.pending.clear()
            self.alignment = None
            self.start = None
            self.last_estimate = None
            self.n = 0
            self.squared = self.angle_squared = 0.
            self.latest = None
            self.rows.clear()
            print('Timestamp reset: starting a new aligned segment.', flush=True)

        def on_truth(self, msg):
            t = stamp(msg)
            if self.truth and t < self.truth[-1][0]:
                self.reset()
            if not self.truth or t > self.truth[-1][0]:
                self.truth.append((t, *pose(msg)))
            self.process()

        def on_estimate(self, msg):
            t = stamp(msg)
            if self.last_estimate is not None and t < self.last_estimate:
                self.reset()
            self.last_estimate = t
            self.pending.append((t, *pose(msg)))
            self.process()

        def process(self):
            if len(self.truth) < 2:
                return
            times = [s[0] for s in self.truth]
            while self.pending:
                t, pe, re = self.pending[0]
                if t > times[-1]:
                    break
                self.pending.popleft()
                j = bisect.bisect_left(times, t)
                if j == 0 or times[j] - times[j-1] > .15:
                    self.dropped += 1
                    continue
                pt, rt = interpolate(self.truth[j-1], self.truth[j], t)
                if self.alignment is None:
                    rotation = rt * re.inv()
                    self.alignment = rotation, pt - rotation.apply(pe)
                    self.start = t
                    print('Aligned first synchronized IMU poses; tracking subsequent drift.', flush=True)
                rotation, translation = self.alignment
                delta = rotation.apply(pe) + translation - pt
                error = float(np.linalg.norm(delta))
                angle = float(np.degrees((rt.inv() * rotation * re).magnitude()))
                self.n += 1
                self.squared += error**2
                self.angle_squared += angle**2
                rmse = math.sqrt(self.squared / self.n)
                angle_rmse = math.sqrt(self.angle_squared / self.n)
                self.latest = error, angle, rmse, angle_rmse
                self.writer.writerow([self.segment, t, t-self.start, *delta, error, angle, rmse, angle_rmse])
                self.rows.append((t-self.start, error, angle))
                self.last_pair_wall = time.monotonic()

        def report(self):
            self.file.flush()
            if self.latest:
                e, a, rmse, ar = self.latest
                age = time.monotonic() - self.last_pair_wall
                print(f'position={e:.3f} m | attitude={a:.2f} deg | '
                      f'RMSE={rmse:.3f} m / {ar:.2f} deg | '
                      f'n={self.n} unmatched={self.dropped} last={age:.1f}s ago', flush=True)
            else:
                print(f'Waiting for synchronized data: truth={len(self.truth)}, '
                      f'estimate queue={len(self.pending)}; timestamps must overlap.', flush=True)
            if args.plot:
                if self.rows:
                    data = np.asarray(self.rows)
                    for i, (line, ax) in enumerate(zip(self.lines, self.axes)):
                        line.set_data(data[:, 0], data[:, i+1])
                        ax.relim()
                        ax.autoscale_view()
                    self.fig.canvas.draw_idle()
                self.plt.pause(.001)

    rclpy.init()
    node = Compare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        if args.plot and node.rows:
            node.fig.savefig(str(Path(args.csv).with_suffix('.png')))
        node.file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
