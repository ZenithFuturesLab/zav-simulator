#!/usr/bin/env python3
"""SITL-only takeoff/hover/landing acceptance using the real flight manager.

Requires an already-running simulator and FAST-LIVO safety_experiment:=True
for jump tests. Never force-disarms a vehicle. Writes telemetry as JSONL.
"""
import argparse
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sitl', action='store_true', required=True)
    parser.add_argument('--fault', choices=('normal', 'position-step', 'attitude-step'), default='normal')
    parser.add_argument('--timeout', type=float, default=240)
    parser.add_argument('--log', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    if args.log.exists():
        parser.error('Log already exists; choose a new --log path')
    import rclpy
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import VehicleLandDetected, FailsafeFlags, VehicleCommandAck, VehicleCommand
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    from zav_control.zav_flight_manager import ZAVFlightManager

    args.log.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = ZAVFlightManager()
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    observed = dict(truth=None, truth_stamp=None, landed=None, safety=None, flags=None, ack=None)
    def truth(msg):
        observed['truth'] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        observed['truth_stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
    def safety(msg):
        observed['safety'] = json.loads(msg.data)
    def flags(msg):
        observed['flags'] = {key: getattr(msg, key) for key in (
            'local_position_invalid', 'local_altitude_invalid', 'attitude_invalid',
            'offboard_control_signal_lost', 'battery_warning')}
    def ack(msg):
        observed['ack'] = dict(command=msg.command, result=msg.result)
    node.create_subscription(Odometry, '/drone0/ground_truth/odometry', truth, qos)
    node.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected',
                             lambda msg: observed.update(landed=msg.landed), qos)
    node.create_subscription(String, '/zav/lio_safety/status', safety, retained)
    node.create_subscription(FailsafeFlags, '/fmu/out/failsafe_flags', flags, qos)
    node.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', ack, qos)
    injector = node.create_publisher(String, '/zav/lio_safety/test/command', 10)
    stage, start, last_log, hover_stamp = 'preflight', time.monotonic(), 0., None
    ground_z = None
    airborne = False
    last_arm_request = -float("inf")
    arm_attempts = 0
    normal_baro_off_verified = False
    with args.log.open('x', encoding='utf-8', buffering=1) as stream:
        try:
            while time.monotonic()-start < args.timeout:
                rclpy.spin_once(node, timeout_sec=.01)
                now = time.monotonic()
                health = observed['safety'] or {}
                if stage == 'preflight':
                    if (now-start > 2 and observed['truth'] is not None
                            and node.position_ready and health.get('ready') and not node.is_armed):
                        if args.fault != 'normal' and injector.get_subscription_count() == 0:
                            raise RuntimeError('No shadow fault relay; launch safety_experiment:=True')
                        ground_z = observed['truth'][2]
                        node.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
                        stage = 'offboard_ready'
                elif stage == 'offboard_ready' and node.nav_state == 14:
                    node.update_teleop('T')
                    last_arm_request, arm_attempts = now, 1
                    stage = 'takeoff'
                elif stage == 'takeoff':
                    if node.is_armed and health.get('baro_phase') == 'OFF_CONFIRMED':
                        normal_baro_off_verified = True
                    if not node.is_armed and now-last_arm_request > 2:
                        if arm_attempts >= 3:
                            raise RuntimeError('PX4 did not arm after three requests; inspect arming checks')
                        node.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.)
                        last_arm_request, arm_attempts = now, arm_attempts+1
                    airborne |= node.is_armed and observed['truth'][2]-ground_z > .5
                    if airborne and node.current_alt >= 1.9 and node.nav_state == 14:
                        hover_stamp = observed['truth_stamp']
                        stage = 'hover'
                elif stage == 'hover' and observed['truth_stamp']-hover_stamp >= 2.:
                    if health.get('baro_phase') != 'OFF_CONFIRMED':
                        raise RuntimeError('Normal-flight barometer fusion was not confirmed OFF')
                    if args.fault == 'normal':
                        node.update_teleop('L')
                    else:
                        injector.publish(String(data=json.dumps(dict(fault=args.fault, duration=5))))
                    stage = 'landing'
                if now-last_log >= 1.:
                    record = dict(elapsed_s=now-start, stage=stage, armed=node.is_armed,
                                  nav_state=node.nav_state, altitude=node.current_alt,
                                  position=[node.current_x, node.current_y], **observed)
                    stream.write(json.dumps(record)+'\n')
                    print(json.dumps(record), flush=True)
                    last_log = now
                if stage not in ('preflight', 'offboard_ready') and node.nav_state == 13:
                    raise RuntimeError('PX4 entered flight termination; landing failed')
                if (stage == 'landing' and airborne and not node.is_armed and observed['landed']
                        and abs(observed['truth'][2]-ground_z) < .3
                        and (args.fault == 'normal' or health.get('landing_complete'))):
                    if args.fault != 'normal':
                        expected = {'position-step': 'position_residual', 'attitude-step': 'attitude_residual'}[args.fault]
                        if not health.get('latched') or expected not in health.get('reasons', []):
                            raise RuntimeError('Landed without the expected motion-jump latch')
                    if not normal_baro_off_verified:
                        raise RuntimeError('No proof of normal flight with barometer fusion OFF')
                    if args.fault != 'normal' and (
                            not health.get('baro_transition_confirmed') or health.get('baro_transition_timed_out')):
                        raise RuntimeError('Landed without a timely confirmed barometer transition')
                    if args.fault == 'normal' and health.get('latched'):
                        raise RuntimeError('Landed, but an unexpected odometry jump triggered the failsafe')
                    stream.write(json.dumps(dict(event='PASS', elapsed_s=now-start,
                                                  armed=node.is_armed, nav_state=node.nav_state,
                                                  ground_z=ground_z,
                                                  normal_baro_off_verified=normal_baro_off_verified,
                                                  **observed))+'\n')
                    print('PASS: airborne -> landing -> ground contact -> disarmed', flush=True)
                    return 0
            raise RuntimeError(f'Timeout during {stage}; armed={node.is_armed}, nav={node.nav_state}')
        except (Exception, KeyboardInterrupt) as error:
            stream.write(json.dumps(dict(event='FAIL', elapsed_s=time.monotonic()-start,
                                          stage=stage, reason=str(error), armed=node.is_armed,
                                          nav_state=node.nav_state, **observed))+'\n')
            raise
        finally:
            if node.is_armed:
                node.update_teleop('L')
                end = time.monotonic()+3
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.02)
            node.multi_key.stop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
