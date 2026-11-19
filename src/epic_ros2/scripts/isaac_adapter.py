#!/usr/bin/env python3
"""Convert LIO IMU odometry for EPIC and forward EPIC commands to ZAV/PX4."""
import copy
import json
import math
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Empty
from visualization_msgs.msg import Marker
from rosgraph_msgs.msg import Clock
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, TrajectorySetpoint
from epic_ros2.msg import PositionCommand, TakeoffLand
from epic_frames import imu_to_body, command_alignment, project_command, project_trajectory


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def yaw(quaternion):
    x, y, z, w = quaternion
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class IsaacAdapter(Node):
    def __init__(self):
        super().__init__('epic_isaac_adapter')
        defaults = {
            'trajectory_feedforward': False,
            'input_odom': '/aft_mapped_to_init',
            'input_hf_odom': '/LIVO2/imu_propagate',
            'maximum_tracking_error': 1.2,
            'imu_mount_position': [.03887666530174469, .02329, .15181667913591162],
            'imu_mount_quaternion': [0., .1908089953765448, 0., .981627183447664],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, descriptor=ParameterDescriptor(read_only=(name == 'trajectory_feedforward')))
        self.mount_p = np.array(self.get_parameter('imu_mount_position').value)
        self.mount_q = np.array(self.get_parameter('imu_mount_quaternion').value)
        self.enabled = False
        self.land_requested = False
        self.anchor = self.body = self.px4 = None
        self.body_t = -1.
        self.armed = False
        self.sim_time = self.heartbeat_wall = self.command_wall = 0.
        self.state = 'WAIT_INPUT'
        self.tracking = None
        sensor = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.odom_pub = self.create_publisher(Odometry, '/epic/odom', 20)
        self.goal_pub = self.create_publisher(PoseStamped, '/zav/goal_pose', 10)
        self.land_pub = self.create_publisher(Bool, '/zav/epic/land_requested', 10)
        self.create_subscription(TakeoffLand, '/px4ctrl/takeoff_land', self.on_land, 10)
        self.status_pub = self.create_publisher(String, '/epic/status', retained)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/zav/epic_trajectory_setpoint_ned', 10
        ) if self.get_parameter('trajectory_feedforward').value else None
        self.clock_pub = self.create_publisher(
            Clock, '/clock', QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry, self.get_parameter('input_odom').value, self.on_odom, sensor)
        self.create_subscription(Odometry, self.get_parameter('input_hf_odom').value, self.on_hf, sensor)
        self.create_subscription(Imu, '/drone0/mid360/imu', self.on_clock, sensor)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_px4, sensor)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.on_vehicle_status, sensor)
        self.create_subscription(Bool, '/epic/control_enabled', self.on_enable, 10)
        self.create_subscription(Empty, '/epic/heartbeat', self.on_heartbeat, 10)
        self.create_subscription(Marker, '/epic/state', self.on_state, 10)
        self.create_subscription(PositionCommand, '/epic/position_cmd', self.on_command, 20)
        self.create_subscription(String, '/zav/lio_safety/status', self.on_safety, retained)
        self.create_timer(.5, self.publish_status)

    def on_clock(self, message):
        t = stamp(message)
        if t < self.sim_time - .1:
            self.anchor = self.body = None
            self.body_t = -1.
        self.sim_time = t
        self.clock_pub.publish(Clock(clock=message.header.stamp))

    def convert(self, message):
        odom = copy.deepcopy(message)
        p, q = message.pose.pose.position, message.pose.pose.orientation
        v, w = message.twist.twist.linear, message.twist.twist.angular
        position, attitude, velocity, angular = imu_to_body(
            [p.x, p.y, p.z], [q.x, q.y, q.z, q.w],
            [v.x, v.y, v.z], [w.x, w.y, w.z], self.mount_p, self.mount_q)
        if not np.isfinite(np.r_[position, attitude, velocity, angular]).all():
            raise ValueError('Nonfinite LIO state')
        odom.child_frame_id = 'epic_body'
        for field, values in ((odom.pose.pose.position, position),
                              (odom.pose.pose.orientation, attitude),
                              (odom.twist.twist.linear, velocity),
                              (odom.twist.twist.angular, angular)):
            for axis, value in zip('xyzw', values):
                setattr(field, axis, float(value))
        return odom

    def on_odom(self, message):
        odom = self.on_hf(message)
        if odom is not None:
            # Preserve the LIO measurement stamp; EPIC synchronizes it with the raw cloud.
            self.odom_pub.publish(odom)

    def on_hf(self, message):
        try:
            odom = self.convert(message)
        except ValueError:
            return None
        t = stamp(odom)
        if t >= self.body_t:
            self.body, self.body_t = odom, t
        return odom

    def on_px4(self, message):
        if message.xy_valid and message.z_valid and math.isfinite(message.heading):
            self.px4 = ([message.x, message.y, -message.z], message.heading)

    def on_vehicle_status(self, message):
        self.armed = message.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def on_heartbeat(self, message):
        self.heartbeat_wall = time.monotonic()

    def on_state(self, message):
        self.state = message.text

    def hold(self):
        if not self.armed or self.px4 is None:
            return
        position, heading = self.px4
        if self.trajectory_pub is not None:
            out = TrajectorySetpoint()
            out.timestamp = time.time_ns() // 1000
            out.position = [float(position[0]), float(position[1]), -float(position[2])]
            out.velocity = out.acceleration = [0., 0., 0.]
            out.yaw, out.yawspeed = float(heading), 0.
            self.trajectory_pub.publish(out)
        else:
            self.publish_goal(position, heading)

    def publish_goal(self, position, heading):
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'zav_neu'
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, position)
        out.pose.orientation.z, out.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
        self.goal_pub.publish(out)

    def request_land(self, reason):
        if not self.armed or not self.enabled and not self.land_requested:
            return
        if not self.land_requested:
            self.get_logger().warn('EPIC stopped; PX4 landing requested: ' + reason)
        self.land_requested = True
        self.enabled, self.anchor = False, None
        self.land_pub.publish(Bool(data=True))

    def on_land(self, message):
        if message.takeoff_land_cmd == TakeoffLand.LAND:
            self.request_land('planner safety/repeated planning failure')

    def on_enable(self, message):
        if message.data and self.land_requested and self.armed:
            return
        if message.data and not self.armed:
            self.land_requested = False
        if self.enabled and not message.data:
            self.hold()
        self.enabled, self.anchor = message.data, None

    def on_safety(self, message):
        try:
            if json.loads(message.data).get('latched'):
                self.enabled, self.anchor = False, None
        except (ValueError, TypeError):
            pass

    def on_command(self, message):
        self.command_wall = time.monotonic()
        if not self.enabled or not self.armed or self.body is None or self.px4 is None:
            return
        if self.trajectory_pub is not None and message.header.frame_id != 'world':
            return
        if (self.sim_time - self.body_t > .1 or time.monotonic() - self.heartbeat_wall > 1.
                or not -.01 <= self.sim_time - stamp(message) <= .5):
            return
        p, q = self.body.pose.pose.position, self.body.pose.pose.orientation
        target = message.position
        goal = np.array([target.x, target.y, target.z])
        body = np.array([p.x, p.y, p.z])
        if not np.isfinite(np.r_[goal, message.yaw]).all():
            return
        self.tracking = float(np.linalg.norm(goal - body))
        if self.anchor is None:
            self.anchor = command_alignment(body, yaw([q.x, q.y, q.z, q.w]), *self.px4)
        if self.tracking > self.get_parameter('maximum_tracking_error').value:
            self.request_land('tracking error exceeded limit')
            return
        if self.trajectory_pub is not None:
            v, a = message.velocity, message.acceleration
            if not np.isfinite([v.x, v.y, v.z, a.x, a.y, a.z, message.yaw_dot]).all():
                self.request_land('nonfinite trajectory command')
                return
            position, velocity, acceleration, heading, rate = project_trajectory(
                goal, [v.x, v.y, v.z], [a.x, a.y, a.z], message.yaw, message.yaw_dot, self.anchor)
            out = TrajectorySetpoint()
            out.timestamp = time.time_ns() // 1000
            out.position, out.velocity, out.acceleration = position.tolist(), velocity.tolist(), acceleration.tolist()
            out.yaw, out.yawspeed = float(heading), float(rate)
            self.trajectory_pub.publish(out)
        else:
            target, heading = project_command(goal, message.yaw, self.anchor)
            self.publish_goal(target, heading)

    def publish_status(self):
        heartbeat_age = time.monotonic() - self.heartbeat_wall
        if self.land_requested and self.armed:
            self.land_pub.publish(Bool(data=True))
        if self.enabled and (heartbeat_age > 1. or
                             (self.command_wall and time.monotonic() - self.command_wall > 1.) or
                             self.sim_time - self.body_t > .2):
            self.request_land('command/heartbeat/odometry timeout')
        data = dict(state=self.state, control_enabled=self.enabled, armed=self.armed,
                    land_requested=self.land_requested,
                    tracking_error_m=self.tracking, odom_age_sim_s=self.sim_time - self.body_t
                    if self.body else None, heartbeat_age_wall_s=heartbeat_age)
        self.status_pub.publish(String(data=json.dumps(data)))


def main():
    rclpy.init()
    node = IsaacAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
