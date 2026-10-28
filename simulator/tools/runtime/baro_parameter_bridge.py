#!/usr/bin/env python3
"""Set/read back EKF2_BARO_CTRL using PX4's bytewise MAVLink parameter protocol.

SITL default uses the unused onboard-payload link (PX4 14280 -> local 14030),
leaving the standard offboard/MAVSDK port 14540 available. Only this one named
parameter is writable. Disabling is allowed only while PX4 is disarmed.
"""
import argparse
import fcntl
import json
import os
import struct
import time

PARAMETER = 'EKF2_BARO_CTRL'
INT32 = 6  # MAV_PARAM_TYPE_INT32; PX4 uses bytewise encoding.


def encode_int32(value):
    return struct.unpack('<f', struct.pack('<i', value))[0]


def decode_int32(value):
    return struct.unpack('<i', struct.pack('<f', value))[0]


class ParameterLink:
    def __init__(self, connection):
        self.connection = connection
        self.target = None
        self.value = None
        self.value_rx = None
        self.heartbeat_rx = None
        self.last_request = -float('inf')
        self.set_count = 0

    def poll(self, now):
        for _ in range(100):
            msg = self.connection.recv_match(blocking=False)
            if msg is None:
                break
            if self.target is None or (msg.get_srcSystem(), msg.get_srcComponent()) != self.target:
                continue
            if msg.get_type() == 'HEARTBEAT':
                self.heartbeat_rx = now
            elif (msg.get_type() == 'PARAM_VALUE' and msg.param_id.rstrip('\0') == PARAMETER
                  and msg.param_type == INT32):
                value = decode_int32(msg.param_value)
                if value in (0, 1):
                    self.value, self.value_rx = value, now

    def tick(self, desired, armed, now):
        self.poll(now)
        if (self.target is None or self.heartbeat_rx is None
                or now-self.heartbeat_rx > 3 or now-self.last_request < .5):
            return
        system, component = self.target
        if self.value != desired and (desired == 1 or not armed):
            self.connection.mav.param_set_send(system, component, PARAMETER.encode(),
                                                encode_int32(desired), INT32)
            self.set_count += 1
        self.connection.mav.param_request_read_send(system, component, PARAMETER.encode(), -1)
        self.last_request = now

    def snapshot(self, desired, now):
        fresh = self.value_rx is not None and now-self.value_rx < 3
        return dict(parameter=PARAMETER, desired=desired, value=self.value,
                    confirmed=fresh and self.value == desired,
                    value_age_s=now-self.value_rx if self.value_rx is not None else None,
                    set_count=self.set_count, target=list(self.target) if self.target else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--connection', default='udpin:127.0.0.1:14030')
    args, ros_args = parser.parse_known_args()
    if ros_args and ros_args[0] != '--ros-args':
        parser.error('Unknown arguments: '+ ' '.join(ros_args))
    lock = open(f'/tmp/zav-baro-parameter-{os.getuid()}.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.exit(1, 'A barometer parameter bridge is already running.\n')
    from pymavlink import mavutil
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import Bool, String
    from px4_msgs.msg import VehicleStatus
    rclpy.init(args=ros_args)
    node = rclpy.create_node('zav_baro_parameter_bridge')
    connection = mavutil.mavlink_connection(args.connection, source_system=255, source_component=191)
    link = ParameterLink(connection)
    state = dict(desired=0, armed=True, status_rx=None, status_stamp=None)
    retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(String, '/zav/lio_safety/baro_parameter', retained)

    def request(msg):
        # A failure request stays enabled until the whole session restarts.
        if msg.data:
            state['desired'] = 1

    def status(msg):
        target = (msg.system_id, msg.component_id)
        if (0 in target or (link.target is not None and target != link.target)
                or (state['status_stamp'] is not None and msg.timestamp <= state['status_stamp'])):
            return
        link.target = target
        state.update(armed=msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
                     status_rx=time.monotonic(), status_stamp=msg.timestamp)

    node.create_subscription(Bool, '/zav/lio_safety/baro_enable', request, retained)
    node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', status,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def tick():
        now = time.monotonic()
        if state['status_rx'] is not None and now-state['status_rx'] < 2:
            link.tick(state['desired'], state['armed'], now)
        else:
            link.poll(now)
        publisher.publish(String(data=json.dumps(link.snapshot(state['desired'], now))))

    node.create_timer(.1, tick)
    node.get_logger().info(f'Baro parameter link: {args.connection}; normal=0, failsafe=1')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        connection.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()


if __name__ == '__main__':
    main()
