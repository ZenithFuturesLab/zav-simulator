#!/usr/bin/env python3
"""SITL shadow faults: corrupt monitor copies, never publish fake /fmu telemetry.

With safety_experiment:=True these copies can trigger REAL NAV_LAND requests.
"""

import argparse
import copy
import json
import math
import os
from pathlib import Path
import time


PREFIX = '/zav/lio_safety/test/'
FAULTS = ('position-step', 'attitude-step')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('relay', help='Started by safety_experiment:=True')
    trigger = commands.add_parser('trigger', help='Inject a monitor fault and observe landing')
    trigger.add_argument('fault', choices=FAULTS)
    trigger.add_argument('--sitl', action='store_true', required=True,
                         help='Confirm this graph is connected only to the simulator')
    trigger.add_argument('--duration', type=float, default=3.0)
    trigger.add_argument('--observe', type=float, default=30.0,
                         help='Maximum time to wait for landing completion')
    trigger.add_argument('--discovery-timeout', type=float, default=10.0,
                         help='Seconds to discover the experiment relay and safety status')
    trigger.add_argument('--log', type=Path, help='Optional new JSONL experiment log')
    trigger.add_argument('--expect', choices=('landing', 'no-failsafe'), default='landing',
                         help='Expected outcome; no-failsafe checks transient-fault recovery')
    args, ros_args = parser.parse_known_args()
    if ros_args and ros_args[0] != '--ros-args':
        parser.error('Unknown arguments: ' + ' '.join(ros_args))
    if args.command == 'trigger' and (not math.isfinite(args.duration) or not 0 < args.duration <= 30
                                      or not math.isfinite(args.observe) or args.observe <= 0
                                      or not math.isfinite(args.discovery_timeout)
                                      or args.discovery_timeout <= 0):
        parser.error('duration must be in (0, 30], observe and discovery-timeout must be positive')

    # TCP without multicast needs a direct discovery peer for the relay itself.
    # Its old ephemeral listener was invisible to a separate trigger process.
    # Select before importing/initializing ROS; leave native Fast DDS unchanged.
    if os.environ.get('ZAV_ROS_TRANSPORT') == 'cyclone_tcp':
        profile = ('cyclonedds_experiment_relay.xml' if args.command == 'relay'
                   else 'cyclonedds_experiment_trigger.xml')
        os.environ['CYCLONEDDS_URI'] = (
            Path(__file__).resolve().parents[1] / 'rviz' / profile).as_uri()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String

    rclpy.init(args=ros_args)
    node = rclpy.create_node('lio_safety_experiment_' + args.command)
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
    stream = None
    try:
        if args.command == 'relay':
            state = dict(fault=None, until=0.0)

            def receive_command(msg):
                try:
                    request = json.loads(msg.data)
                    duration = float(request['duration'])
                    if request['fault'] not in FAULTS or not math.isfinite(duration) or not 0 < duration <= 30:
                        raise ValueError('Invalid fault or duration')
                    state.update(fault=request['fault'], until=time.monotonic() + duration)
                    node.get_logger().warning(f'SITL SHADOW FAULT: {request}')
                except (ValueError, KeyError, TypeError) as error:
                    node.get_logger().error(str(error))

            node.create_subscription(String, PREFIX + 'command', receive_command, 10)

            def route(kind, publisher):
                def callback(msg):
                    fault = state['fault'] if time.monotonic() < state['until'] else None
                    out = copy.deepcopy(msg)
                    if kind == 'odom' and fault == 'position-step':
                        out.pose.pose.position.x += 3.0
                    if kind == 'odom' and fault == 'attitude-step':
                        q = out.pose.pose.orientation
                        c, s = math.cos(math.pi/4), math.sin(math.pi/4)
                        q.w, q.x, q.y, q.z = c*q.w-s*q.z, c*q.x-s*q.y, c*q.y+s*q.x, c*q.z+s*q.w
                    publisher.publish(out)
                return callback

            routes = [(Odometry, '/LIVO2/imu_propagate', 'odom')]
            for typ, source, destination in routes:
                pub = node.create_publisher(typ, PREFIX + destination, qos)
                node.create_subscription(typ, source, route(destination, pub), qos)
            node.get_logger().warning('SITL shadow relay ready. Triggering faults can command a real landing!')
            rclpy.spin(node)
        else:
            if args.log:
                args.log.parent.mkdir(parents=True, exist_ok=True)
                stream = args.log.open('x', encoding='utf-8', buffering=1)
            observed = dict(status=None, received=None)
            started = time.monotonic()

            def status(msg):
                data = json.loads(msg.data)
                observed['status'] = data
                observed['received'] = time.monotonic()
                entry = dict(elapsed_s=time.monotonic()-started, **data)
                if stream:
                    stream.write(json.dumps(entry) + '\n')

            retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(String, '/zav/lio_safety/status', status, retained)
            pub = node.create_publisher(String, PREFIX + 'command', 10)
            deadline = time.monotonic() + args.discovery_timeout
            while time.monotonic() < deadline and (
                    pub.get_subscription_count() == 0 or observed['status'] is None):
                rclpy.spin_once(node, timeout_sec=0.05)
            health = observed['status']
            if pub.get_subscription_count() == 0:
                parser.exit(1, 'Experiment relay not discovered on /zav/lio_safety/test/command. '
                            'Start/restart FAST-LIVO with safety_experiment:=True. '
                            'WSL Cyclone TCP relay must listen on port 17452.\n')
            if health is None:
                parser.exit(1, 'Safety status not received on /zav/lio_safety/status. '
                            'Check the safety-enabled lio_px4_bridge (WSL TCP port 17451).\n')
            if not health['armed'] or not health['ready']:
                parser.exit(1, 'Fault refused: simulator must be armed and safety READY. Hover first.\n')
            started = time.monotonic()
            if stream:
                stream.write(json.dumps(dict(event='injection', fault=args.fault,
                                              duration_s=args.duration, expect=args.expect)) + '\n')
            pub.publish(String(data=json.dumps(dict(fault=args.fault, duration=args.duration))))
            print(f'Injected {args.fault} for {args.duration:g}s. Observing landing...', flush=True)
            previous = None
            while time.monotonic() - started < args.observe:
                rclpy.spin_once(node, timeout_sec=0.1)
                health = observed['status']
                if time.monotonic() - observed['received'] > 2.0:
                    print('FAIL: safety status stopped arriving.', flush=True)
                    return 1
                summary = (health['state'], health['latched'], health['landing_mode_confirmed'],
                           health['landing_complete'], health['land_ack'])
                if summary != previous:
                    print(json.dumps(health), flush=True)
                    previous = summary
                if args.expect == 'no-failsafe' and health['latched']:
                    print('FAIL: transient fault unexpectedly latched the failsafe.', flush=True)
                    return 1
                if args.expect == 'landing' and health['latched'] and health['landing_complete']:
                    print('PASS: failsafe latched, landed and disarmed.', flush=True)
                    return 0
            if args.expect == 'no-failsafe' and not health['latched'] and health['ready']:
                print('PASS: transient fault recovered to READY without a failsafe.', flush=True)
                return 0
            print('FAIL: landing completion not confirmed. Inspect status, ACK and PX4 logs.', flush=True)
            return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if stream:
            stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
