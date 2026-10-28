#!/usr/bin/env python3
"""Record FAST-LIVO Odometry messages to CSV without changing their frames."""

import argparse
import csv
from datetime import datetime
from pathlib import Path


FIELDS = [
    'stamp_sec', 'stamp_nanosec', 'timestamp_s', 'frame_id', 'child_frame_id',
    'position_x', 'position_y', 'position_z',
    'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
    'linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z',
    'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
]


def odometry_row(msg):
    """Keep the exact measurement timestamp alongside readable seconds."""
    stamp = msg.header.stamp
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    v, w = msg.twist.twist.linear, msg.twist.twist.angular
    timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    seconds, nanoseconds = divmod(timestamp_ns, 1_000_000_000)
    return [
        stamp.sec, stamp.nanosec, f'{seconds}.{nanoseconds:09d}',
        msg.header.frame_id, msg.child_frame_id,
        p.x, p.y, p.z, q.x, q.y, q.z, q.w,
        v.x, v.y, v.z, w.x, w.y, w.z,
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/aft_mapped_to_init',
                        help='Odometry topic (default: /aft_mapped_to_init)')
    parser.add_argument('--output', type=Path,
                        help='CSV path; existing files are never overwritten')
    args = parser.parse_args()
    output = args.output or (
        Path(__file__).resolve().parents[3] / 'log' / 'odometry'
        / f'fastlivo_{datetime.now():%Y%m%d_%H%M%S_%f}.csv'
    )
    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = None
    count = 0
    try:
        with output.open('x', newline='', encoding='utf-8', buffering=1) as stream:
            writer = csv.writer(stream)
            writer.writerow(FIELDS)
            stream.flush()
            node = Node('zav_odometry_csv_recorder')

            def record(msg):
                nonlocal count
                writer.writerow(odometry_row(msg))
                stream.flush()
                count += 1

            # Compatible with both reliable and best-effort publishers.
            qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
            subscription = node.create_subscription(Odometry, args.topic, record, qos)
            node.get_logger().info(f'Recording {args.topic} -> {output}; Ctrl+C to stop')
            node.create_timer(5.0, lambda: node.get_logger().info(f'Saved {count} rows'))
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f'Saved {count} rows: {output}', flush=True)


if __name__ == '__main__':
    main()
