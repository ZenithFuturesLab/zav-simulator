#!/usr/bin/env python3
"""Follow a growing FAST-LIVO CSV and plot position, attitude and velocity."""

import argparse
from collections import deque
import csv
import math
from pathlib import Path


class CsvTail:
    """Read only appended, complete lines; retain partial writes for next poll."""

    def __init__(self, path):
        self.path = path
        self.offset = 0
        self.pending = b''
        self.fields = None

    def read(self):
        if not self.path.exists():
            return []
        if self.path.stat().st_size < self.offset:
            self.offset = 0
            self.pending = b''
            self.fields = None
        with self.path.open('rb') as stream:
            stream.seek(self.offset)
            chunk = stream.read()
            self.offset = stream.tell()
        lines = (self.pending + chunk).split(b'\n')
        self.pending = lines.pop()
        rows = []
        for line in lines:
            if not line.strip():
                continue
            values = next(csv.reader([line.decode('utf-8').rstrip('\r')]))
            if self.fields is None:
                self.fields = values
                required = {
                    'stamp_sec', 'stamp_nanosec',
                    *(f'position_{axis}' for axis in 'xyz'),
                    *(f'orientation_{axis}' for axis in 'xyzw'),
                    *(f'linear_velocity_{axis}' for axis in 'xyz'),
                }
                missing = required - set(values)
                if missing:
                    raise ValueError(f'Missing CSV columns: {sorted(missing)}')
            elif len(values) == len(self.fields):
                rows.append(dict(zip(self.fields, values)))
        return rows


def sample(row):
    timestamp_ns = int(row['stamp_sec']) * 1_000_000_000 + int(row['stamp_nanosec'])
    position = [float(row[f'position_{axis}']) for axis in 'xyz']
    velocity = [float(row[f'linear_velocity_{axis}']) for axis in 'xyz']
    x, y, z, w = [float(row[f'orientation_{axis}']) for axis in 'xyzw']
    norm = math.hypot(x, y, z, w)
    if not math.isfinite(norm) or norm < 1e-12:
        angles = [math.nan] * 3
    else:
        x, y, z, w = (v / norm for v in (x, y, z, w))
        angles = [
            math.degrees(math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))),
            math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w*y - z*x))))),
            math.degrees(math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))),
        ]
    return timestamp_ns, position, angles, velocity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='?', type=Path,
                        help='CSV path; default: newest log/odometry/fastlivo_*.csv')
    parser.add_argument('--interval', type=int, default=200, help='Refresh period in ms')
    parser.add_argument('--max-points', type=int, default=10000,
                        help='Maximum displayed samples (default: 10000)')
    args = parser.parse_args()
    if args.interval <= 0 or args.max_points < 2:
        parser.error('--interval must be positive and --max-points must be at least 2')
    path = args.csv
    if path is None:
        directory = Path(__file__).resolve().parents[3] / 'log' / 'odometry'
        candidates = list(directory.glob('fastlivo_*.csv'))
        if not candidates:
            parser.error('No odometry CSV found. Start ./record_odometry.sh or specify a CSV path.')
        path = max(candidates, key=lambda p: p.stat().st_mtime_ns)
    path = path.expanduser().resolve()

    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError:
        parser.exit(1, 'matplotlib is required: sudo apt install python3-matplotlib\n')

    tail = CsvTail(path)
    data = deque(maxlen=args.max_points)
    origin = None
    count = 0
    fig = plt.figure(figsize=(13, 8))
    position_ax = fig.add_subplot(2, 2, 1)
    attitude_ax = fig.add_subplot(2, 2, 2)
    velocity_ax = fig.add_subplot(2, 2, 3)
    trajectory_ax = fig.add_subplot(2, 2, 4, projection='3d')
    groups = []
    for ax, title, unit, labels in (
        (position_ax, 'Position', 'm', ('x', 'y', 'z')),
        (attitude_ax, 'Attitude (ZYX Euler)', 'deg', ('roll', 'pitch', 'yaw')),
        (velocity_ax, 'Linear velocity (child frame)', 'm/s', ('vx', 'vy', 'vz')),
    ):
        groups.append([ax.plot([], [], label=label)[0] for label in labels])
        ax.set(title=title, xlabel='Measurement time since first sample (s)', ylabel=unit)
        ax.grid(True)
        ax.legend(loc='upper right')
    trajectory, = trajectory_ax.plot([], [], [], label='trajectory')
    marker, = trajectory_ax.plot([], [], [], 'ro', label='latest')
    trajectory_ax.set(title='XYZ trajectory', xlabel='x (m)', ylabel='y (m)',
                      zlabel='z (m)')
    trajectory_ax.set_box_aspect((1, 1, 1))
    trajectory_ax.grid(True)
    trajectory_ax.legend()
    title = fig.suptitle(str(path))
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    def update(_):
        nonlocal origin, count
        try:
            rows = tail.read()
        except (OSError, ValueError, UnicodeError) as error:
            title.set_text(f'{path.name}: {error}')
            return
        for row in rows:
            try:
                timestamp, position, angles, velocity = sample(row)
            except (ValueError, KeyError, OverflowError):
                continue
            # Simulator reset: begin a new time series instead of drawing across it.
            if data and timestamp < data[-1][0]:
                data.clear()
                origin = None
            if origin is None:
                origin = timestamp
            data.append((timestamp, position, angles, velocity))
            count += 1
        title.set_text(f'{path.name} | {count} samples | '
                       f'{"waiting for data" if not data else "following CSV"}')
        if not data or not rows:
            return
        times = [(item[0] - origin) / 1e9 for item in data]
        for group_index, lines in enumerate(groups, start=1):
            for component, line in enumerate(lines):
                line.set_data(times, [item[group_index][component] for item in data])
        coordinates = [[item[1][axis] for item in data] for axis in range(3)]
        trajectory.set_data_3d(*coordinates)
        marker.set_data_3d(*([value] for value in data[-1][1]))
        # Equal axis spans preserve distances, including flat or stationary paths.
        bounds = []
        for values in coordinates:
            finite = [value for value in values if math.isfinite(value)]
            bounds.append((min(finite), max(finite)) if finite else (-0.5, 0.5))
        span = max(0.1, *(high - low for low, high in bounds)) * 1.1
        for setter, (low, high) in zip(
                (trajectory_ax.set_xlim, trajectory_ax.set_ylim, trajectory_ax.set_zlim),
                bounds):
            center = (low + high) / 2
            setter(center - span / 2, center + span / 2)
        for ax in (position_ax, attitude_ax, velocity_ax):
            ax.relim()
            ax.autoscale_view()

    animation = FuncAnimation(fig, update, interval=args.interval, cache_frame_data=False)
    print(f'Following {path}\nClose the plot window or press Ctrl+C to stop.', flush=True)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        plt.close(fig)


if __name__ == '__main__':
    main()
