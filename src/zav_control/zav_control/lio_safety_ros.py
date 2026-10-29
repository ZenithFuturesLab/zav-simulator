"""ROS wiring for the LIO safety gate and acknowledged PX4 landing requests."""

import json
import math
import time

from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    EstimatorAidSource1d, VehicleCommand, VehicleCommandAck, VehicleLandDetected,
    VehicleLocalPosition, VehicleStatus,
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String

from zav_control.lio_safety import LioSafety, SafetyConfig


class LioSafetyROS:
    def __init__(self, node, args):
        self.node, self.args = node, args
        self.policy = LioSafety(SafetyConfig(
            position_slack=args.safety_position_slack,
            angle_slack=math.radians(args.safety_angle_slack_deg),
            prediction_horizon=args.safety_prediction_horizon,
        ))
        self.status = None
        self.status_rx = None
        self.landed = None
        self.landed_rx = None
        self.last_command = -math.inf
        self.command_count = 0
        self.last_ack = None
        self.last_log = -math.inf
        self.previous_state = None
        self.snapshot = dict(latched=False)
        self.baro_parameter = None
        self.baro_parameter_rx = None
        self.baro_aid = None
        self.baro_aid_rx = None
        self.baro_good_rx = None
        self.local_position = None
        self.local_position_rx = None
        self.baro_start = None
        self.baro_transition_confirmed = False
        self.baro_transition_timed_out = False
        self.baro_activation_seconds = None
        self.shadow_input = args.safety_odom_topic != args.odom_topic
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.command_pub = node.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.latch_pub = node.create_publisher(Bool, '/zav/lio_safety/land_requested', retained)
        self.ready_pub = node.create_publisher(Bool, '/zav/lio_safety/ready', retained)
        self.status_pub = node.create_publisher(String, '/zav/lio_safety/status', retained)
        self.baro_request_pub = node.create_publisher(Bool, '/zav/lio_safety/baro_enable', retained)
        node.create_subscription(String, '/zav/lio_safety/baro_parameter', self.on_baro_parameter, retained)
        node.create_subscription(EstimatorAidSource1d, '/fmu/out/estimator_aid_src_baro_hgt',
                                 self.on_baro_aid, qos)
        node.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self.on_local_position, qos)
        if self.shadow_input:
            node.create_subscription(Odometry, args.safety_odom_topic, self.inspect, qos)
        node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.on_status, qos)
        node.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.on_ack, qos)
        node.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.on_land, qos)
        node.create_timer(0.05, self.tick)
        self.tick()
        node.get_logger().info(
            f'LIO motion-continuity safety enabled; monitor={args.safety_odom_topic}; '
            f'action={"LAND" if args.publish else "DRY-RUN"}')

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def on_status(self, msg):
        if self.status is not None and msg.timestamp <= self.status.timestamp:
            return
        self.status, self.status_rx = msg, time.monotonic()
        self.policy.set_armed(msg.arming_state == VehicleStatus.ARMING_STATE_ARMED, self.status_rx)

    def on_land(self, msg):
        if self.landed is None or msg.timestamp > self.landed.timestamp:
            self.landed, self.landed_rx = msg, time.monotonic()

    def on_baro_parameter(self, msg):
        try:
            data = json.loads(msg.data)
            if (not isinstance(data, dict) or data.get('parameter') != 'EKF2_BARO_CTRL'
                    or data.get('value') not in (0, 1)
                    or not data.get('confirmed') or self.status is None
                    or data.get('target') != [self.status.system_id, self.status.component_id]):
                return
            self.baro_parameter, self.baro_parameter_rx = data, time.monotonic()
        except (ValueError, TypeError):
            return

    def on_baro_aid(self, msg):
        if msg.estimator_instance != 0:
            return  # This profile runs a single EKF instance.
        if self.baro_aid is not None and msg.timestamp_sample <= self.baro_aid.timestamp_sample:
            return
        self.baro_aid, self.baro_aid_rx = msg, time.monotonic()
        if (msg.fusion_enabled and msg.fused and not msg.innovation_rejected
                and math.isfinite(msg.test_ratio) and 0 <= msg.test_ratio <= 1
                and msg.time_last_fuse > 0):
            self.baro_good_rx = self.baro_aid_rx

    def on_local_position(self, msg):
        if self.local_position is None or msg.timestamp > self.local_position.timestamp:
            self.local_position, self.local_position_rx = msg, time.monotonic()

    def on_ack(self, msg):
        if (self.command_count and msg.command == VehicleCommand.VEHICLE_CMD_NAV_LAND
                and msg.target_component == self.args.safety_source_component
                and msg.target_system == self.args.safety_source_system):
            self.last_ack = int(msg.result)
            self.node.get_logger().info(f'LIO failsafe NAV_LAND ack={msg.result}; '
                                         'waiting for vehicle_status mode confirmation')

    def inspect(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        was_latched = self.policy.latched
        previous = self.policy.last_odom
        if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'aft_mapped':
            accepted = self.policy.odometry(self.stamp(msg), (math.nan, 0, 0),
                                             (1, 0, 0, 0), (0, 0, 0), time.monotonic())
        else:
            accepted = self.policy.odometry(self.stamp(msg), (p.x, p.y, p.z),
                                             (q.w, q.x, q.y, q.z), (v.x, v.y, v.z),
                                             time.monotonic())
        if not was_latched and self.policy.latched:
            self.node.get_logger().error('LIO jump evidence: ' + json.dumps({
                'reasons': self.policy.latch_reasons,
                'metrics': self.policy.metrics,
                'previous_stamp': previous[0] if previous else None,
                'stamp': self.stamp(msg),
                'previous_quaternion_wxyz': previous[2] if previous else None,
                'quaternion_wxyz': [q.w, q.x, q.y, q.z],
                'position': [p.x, p.y, p.z],
            }))
        self.tick()
        return accepted and not self.policy.latched

    def allow_forward(self, msg):
        if not self.shadow_input:
            accepted = self.inspect(msg)
        else:
            self.tick()
            accepted = self.policy.residual_error is None
        return accepted and not self.policy.latched

    def tick(self):
        now = time.monotonic()
        stale_status = self.status_rx is None or now - self.status_rx > 1.0
        result = self.policy.evaluate()
        parameter_fresh = (self.baro_parameter_rx is not None
                           and now-self.baro_parameter_rx < 2)
        aid_fresh = self.baro_aid_rx is not None and now-self.baro_aid_rx < 1
        normal_baro_off = (parameter_fresh and self.baro_parameter['value'] == 0
                           and aid_fresh and not self.baro_aid.fusion_enabled)
        if result['latched'] and self.baro_start is None:
            self.baro_start = now
        baro_ready = (self.baro_start is not None and parameter_fresh
                      and self.baro_parameter['value'] == 1
                      and self.baro_parameter_rx >= self.baro_start
                      and aid_fresh and self.baro_aid.fusion_enabled
                      and self.baro_good_rx is not None and self.baro_good_rx >= self.baro_start
                      and now-self.baro_good_rx < 1
                      and self.local_position_rx is not None and self.local_position_rx >= self.baro_start
                      and now-self.local_position_rx < 1
                      and self.local_position.z_valid and self.local_position.v_z_valid
                      and math.isfinite(self.local_position.z) and math.isfinite(self.local_position.vz))
        if baro_ready and not self.baro_transition_confirmed:
            self.baro_transition_confirmed = True
            self.baro_activation_seconds = now-self.baro_start
            self.node.get_logger().info(f'Baro fusion and vertical state confirmed in '
                                         f'{self.baro_activation_seconds:.3f}s; landing enabled')
        if (self.baro_start is not None and not self.baro_transition_confirmed
                and now-self.baro_start >= 3 and not self.baro_transition_timed_out):
            self.baro_transition_timed_out = True
            self.node.get_logger().error('Baro activation not confirmed within 3s; '
                                         'continuing enable retries and requesting best-effort Land')
        if not result['latched'] and self.args.publish:
            result['ready'] &= bool(normal_baro_off)
            if not result['ready'] and result['state'] == 'READY':
                result['state'] = 'WAIT_BARO_OFF'
        landing_mode = (not stale_status and self.status.nav_state in (
            VehicleStatus.NAVIGATION_STATE_AUTO_LAND, VehicleStatus.NAVIGATION_STATE_DESCEND))
        # An accepted ACK alone is not evidence that the vehicle entered Land mode.
        # No corrupted EV is sent while waiting. Never treat a parameter echo as
        # proof of fusion; require a fused baro sample plus valid vertical state.
        transition_done = self.baro_transition_confirmed or self.baro_transition_timed_out
        if (result['latched'] and transition_done and self.policy.armed
                and not stale_status and not landing_mode):
            if now - self.last_command >= 1.0 and self.args.publish:
                self.send_land(now)
        result.update(landing_mode_confirmed=landing_mode,
                      baro_phase=('ACTIVE' if self.baro_transition_confirmed else
                                  'ACTIVATION_TIMEOUT' if self.baro_transition_timed_out else
                                  'ACTIVATING' if result['latched'] else
                                  'OFF_CONFIRMED' if normal_baro_off else 'WAIT_OFF_CONFIRMATION'),
                      baro_parameter_value=(self.baro_parameter['value'] if parameter_fresh else None),
                      baro_fusion_enabled=(bool(self.baro_aid.fusion_enabled) if aid_fresh else None),
                      baro_transition_confirmed=self.baro_transition_confirmed,
                      baro_transition_timed_out=self.baro_transition_timed_out,
                      baro_activation_seconds=self.baro_activation_seconds,
                      nav_state=int(self.status.nav_state) if self.status is not None else None,
                      land_ack=self.last_ack, land_command_count=self.command_count,
                      landing_complete=bool(self.landed is not None and self.landed.landed
                                            and now - self.landed_rx < 1.0
                                            and not stale_status and not self.policy.armed))
        self.snapshot = result
        result['metrics'] = {key: value if math.isfinite(value) else None
                             for key, value in result['metrics'].items()}
        # Publish at 5 Hz; hard faults publish immediately before returning to bridge.
        changed = result['state'] != self.previous_state
        if changed or now - self.last_log >= 0.2:
            if self.args.publish:
                self.baro_request_pub.publish(Bool(data=result['latched']))
                self.latch_pub.publish(Bool(data=result['latched']))
                self.ready_pub.publish(Bool(data=result['ready']))
            self.status_pub.publish(String(data=json.dumps(result, allow_nan=False,
                                                           default=str)))
            self.last_log = now
        if changed:
            self.node.get_logger().warning(f'LIO SAFETY {result["state"]}: {result["reasons"]}')
            self.previous_state = result['state']

    def send_land(self, now):
        msg = VehicleCommand()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds // 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        msg.param4 = msg.param5 = msg.param6 = msg.param7 = math.nan
        msg.target_system, msg.target_component = self.status.system_id, self.status.component_id
        msg.source_system = self.args.safety_source_system
        msg.source_component = self.args.safety_source_component
        msg.from_external = True
        self.command_pub.publish(msg)
        self.last_command = now
        self.command_count += 1
        self.node.get_logger().error(f'LIO FAILSAFE: NAV_LAND request #{self.command_count}: '
                                     f'{self.policy.latch_reasons}')


def add_safety_arguments(parser):
    parser.add_argument('--safety', action='store_true', help='Enable motion-jump landing failsafe')
    parser.add_argument('--safety-odom-topic', default=None,
                        help='Defaults to bridge input; separate monitor topic for shadow experiments')
    parser.add_argument('--safety-position-slack', type=float, default=0.5)
    parser.add_argument('--safety-angle-slack-deg', type=float, default=20.0)
    parser.add_argument('--safety-prediction-horizon', type=float, default=0.25,
                        help='Maximum measurement-time comparison gap; longer gaps only rebaseline')
    parser.add_argument('--safety-source-system', type=int, default=255)
    parser.add_argument('--safety-source-component', type=int, default=191)
