"""ROS integration with all aircraft topics remapped to a unique test namespace.

No command in these tests can reach /fmu/in/vehicle_command.
"""

import argparse
import math
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
import uuid
from unittest.mock import Mock

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('px4_msgs.msg')
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleLandDetected, VehicleStatus, VehicleOdometry, VehicleLocalPosition, EstimatorAidSource1d
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from zav_control import lio_safety_ros as safety_module
from zav_control import zav_flight_manager as manager_module


@pytest.fixture
def harness(monkeypatch):
    topics = [
        '/fmu/in/vehicle_command', '/fmu/in/offboard_control_mode',
        '/fmu/in/trajectory_setpoint', '/fmu/out/vehicle_status',
        '/fmu/in/vehicle_visual_odometry', '/fmu/out/sensor_combined',
        '/fmu/out/timesync_status', '/drone0/mid360/imu',
        '/fmu/out/vehicle_command_ack', '/fmu/out/vehicle_land_detected',
        '/fmu/out/vehicle_local_position', '/aft_mapped_to_init',
        '/LIVO2/imu_propagate', '/zav/lio_safety/status',
        '/zav/lio_safety/ready', '/zav/lio_safety/land_requested',
        '/zav/goal_pose', '/zav/cmd_vel',
        '/zav/lio_safety/baro_enable', '/zav/lio_safety/baro_parameter',
        '/fmu/out/estimator_aid_src_baro_hgt',
        *(f'/zav/lio_safety/test/{name}' for name in (
            'command', 'odom')),
    ]
    prefix = '/test_lio_' + uuid.uuid4().hex
    argv = ['--ros-args']
    for topic in topics:
        argv.extend(['-r', f'{topic}:={prefix}{topic}'])
    rclpy.init(args=argv)
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(safety_module, 'time', SimpleNamespace(monotonic=lambda: clock.now))
    monkeypatch.setattr(manager_module, 'Win32MultiKeyReader', lambda: SimpleNamespace(
        active=False, poll_latest=lambda: 0, stop=lambda: None))
    monkeypatch.setattr(manager_module.ZAVFlightManager, '_print_guide', lambda self: None)
    node = rclpy.create_node('lio_safety_integration')
    parser = argparse.ArgumentParser()
    safety_module.add_safety_arguments(parser)
    args = parser.parse_args([])
    args.publish = True
    args.odom_topic = args.safety_odom_topic = '/LIVO2/imu_propagate'
    guard = safety_module.LioSafetyROS(node, args)
    manager = manager_module.ZAVFlightManager()
    manager.flight_mode = 'OFFBOARD_TRAJ'
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(manager)
    commands = []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(VehicleCommand, '/fmu/in/vehicle_command', commands.append, qos)

    def spin(seconds=0.1):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.005)

    baro = dict(active=False, fused=False)

    def frame(now, *, armed=True, position=None,
              nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD):
        clock.now = now
        status = VehicleStatus()
        status.timestamp = int((now+100)*1e6)
        status.arming_state = VehicleStatus.ARMING_STATE_ARMED if armed else VehicleStatus.ARMING_STATE_STANDBY
        status.nav_state = nav_state
        status.system_id, status.component_id = 1, 1
        guard.on_status(status)
        guard.on_baro_parameter(String(data=json.dumps(dict(parameter='EKF2_BARO_CTRL',
            value=int(baro['active']), confirmed=True, target=[1, 1]))))
        aid = EstimatorAidSource1d()
        aid.timestamp_sample = int((now+10)*1e6)
        aid.fusion_enabled, aid.fused = baro['active'], baro['fused']
        aid.time_last_fuse = int((now+10)*1e6) if aid.fused else 0
        aid.test_ratio = .1
        guard.on_baro_aid(aid)
        local = VehicleLocalPosition()
        local.timestamp = status.timestamp
        local.z_valid = local.v_z_valid = True
        guard.on_local_position(local)
        msg = Odometry()
        msg.header.stamp.sec = int(now+10)
        msg.header.stamp.nanosec = int(round((now+10-int(now+10))*1e9))
        msg.header.frame_id, msg.child_frame_id = 'camera_init', 'aft_mapped'
        msg.pose.pose.position.x = position if position is not None else now
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = 1.0
        return guard.allow_forward(msg)

    try:
        # Discover the retained latch/ready topics and command subscriber first.
        spin(0.4)
        for tick in range(101):
            frame(tick/50, armed=False)
        assert guard.snapshot['ready']
        frame(2.02)
        spin()
        assert not commands and not manager.lio_safety_latched
        yield SimpleNamespace(node=node, guard=guard, manager=manager, commands=commands,
                              frame=frame, spin=spin, clock=clock, ros_args=argv, baro=baro)
    finally:
        executor.shutdown()
        manager.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_land_command_latch_retries_mode_confirmation_and_completion(harness):
    h = harness
    assert not h.frame(2.04, position=5.04)
    h.spin()
    assert h.guard.policy.latched and not h.commands
    # Parameter ACK by itself must not release the landing transition.
    h.baro['active'] = True
    h.frame(2.10)
    h.spin()
    assert not h.commands
    h.baro['fused'] = True
    h.frame(2.12)
    h.spin()
    assert h.guard.snapshot['baro_transition_confirmed'] and h.commands
    cmd = h.commands[0]
    assert cmd.command == VehicleCommand.VEHICLE_CMD_NAV_LAND
    assert math.isnan(cmd.param5) and math.isnan(cmd.param6)
    assert cmd.target_system == cmd.target_component == 1
    assert h.manager.lio_safety_latched and not h.manager.setpoints_enabled
    assert h.manager.update_teleop('T') is True
    assert not h.manager.setpoints_enabled
    setpoint_spy, offboard_spy = Mock(), Mock()
    h.manager.setpoint_pub = SimpleNamespace(publish=setpoint_spy)
    h.manager.offboard_mode_pub = SimpleNamespace(publish=offboard_spy)
    h.manager._control_loop()
    setpoint_spy.assert_not_called()
    offboard_spy.assert_not_called()
    count = h.guard.command_count

    # A denied command must retry; an accepted ACK still needs actual mode feedback.
    ack = VehicleCommandAck()
    ack.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
    ack.target_system, ack.target_component = 255, 191
    ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED
    h.guard.on_ack(ack)
    h.frame(4.1)
    assert h.guard.command_count > count
    ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
    h.guard.on_ack(ack)
    h.frame(5.2)
    assert not h.guard.snapshot['landing_mode_confirmed']
    count = h.guard.command_count
    h.frame(6.3, nav_state=VehicleStatus.NAVIGATION_STATE_AUTO_LAND)
    assert h.guard.snapshot['landing_mode_confirmed']
    assert h.guard.command_count == count
    assert not h.guard.snapshot['landing_complete']
    landed = VehicleLandDetected()
    landed.timestamp, landed.landed = 10_000_000, True
    h.guard.on_land(landed)
    h.frame(6.4, armed=False, nav_state=VehicleStatus.NAVIGATION_STATE_AUTO_LAND)
    assert h.guard.snapshot['landing_complete'] and h.guard.policy.latched


def test_hard_jump_never_forwarded_and_late_manager_observes_retained_latch(harness):
    h = harness
    assert not h.frame(2.04, position=5.04)
    h.spin()
    assert h.guard.policy.latched and h.manager.lio_safety_latched
    assert not h.frame(2.06)
    late = manager_module.ZAVFlightManager()
    # A fresh subscriber must see the retained failure without another transition.
    executor = SingleThreadedExecutor()
    executor.add_node(late)
    try:
        deadline = time.monotonic() + 1
        while not late.lio_safety_latched and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert late.lio_safety_latched and not late.setpoints_enabled
    finally:
        executor.shutdown()
        late.destroy_node()


def test_dry_run_never_commands_on_jump(harness):
    h = harness
    h.guard.args.publish = False
    assert not h.frame(2.04, position=5.04)
    h.spin()
    assert h.guard.policy.latched
    assert not h.commands and not h.manager.lio_safety_latched
    assert 'position_residual' in h.guard.snapshot['reasons']


def test_no_messages_or_stale_vehicle_status_do_not_trigger_landing(harness):
    h = harness
    h.clock.now = 100.
    h.guard.tick()
    h.spin()
    assert not h.guard.policy.latched and not h.commands


def test_experiment_relay_faults_use_only_monitor_copies(harness):
    h = harness
    script = Path(__file__).resolve().parents[3] / 'simulator/tools/runtime/lio_safety_experiment.py'
    proc = subprocess.Popen([sys.executable, str(script), 'relay', *h.ros_args],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    prefix = '/zav/lio_safety/test/'
    received = {'odom': []}
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
    for name, typ in [('odom', Odometry)]:
        h.node.create_subscription(typ, prefix+name, received[name].append, qos)
    command = h.node.create_publisher(String, prefix+'command', 10)
    odom = h.node.create_publisher(Odometry, '/LIVO2/imu_propagate', 10)
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'camera_init', 'aft_mapped'
    msg.pose.pose.orientation.w = 1.0
    try:
        deadline = time.monotonic() + 5
        while command.get_subscription_count() == 0 and time.monotonic() < deadline:
            h.spin(0.05)
        assert command.get_subscription_count() > 0
        h.spin(0.2)
        for fault in ('position-step', 'attitude-step'):
            command.publish(String(data=json.dumps(dict(fault=fault, duration=5))))
            h.spin(0.1)
            for values in received.values():
                values.clear()
            msg.header.stamp.sec = 20
            odom.publish(msg)
            h.spin(0.15)
            if fault == 'position-step':
                assert received['odom'][-1].pose.pose.position.x == 3.0
                assert msg.pose.pose.position.x == 0.0
            elif fault == 'attitude-step':
                assert received['odom'][-1].pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    finally:
        proc.send_signal(signal.SIGINT)
        logs, _ = proc.communicate(timeout=5)
        assert proc.returncode == 0, logs


def test_actual_bridge_forwards_dt_consistent_motion_and_blocks_jump(harness):
    from zav_control.lio_px4_bridge import LioPx4Bridge

    h = harness
    parser = argparse.ArgumentParser()
    safety_module.add_safety_arguments(parser)
    args = parser.parse_args(['--safety'])
    args.publish = True
    args.odom_topic = args.safety_odom_topic = '/LIVO2/imu_propagate'
    args.hz, args.max_lio_age_ms, args.max_imu_age_ms = 50., 250., 100.
    args.max_position_step_m = 1.
    bridge = LioPx4Bridge(args)
    received = []
    h.node.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', received.append,
                               QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        h.spin(0.3)
        now_us = time.time_ns() // 1000
        bridge.latest_imu_stamp_ros_us = bridge.latest_imu_wall_us = now_us
        bridge.latest_imu_mono_us = time.monotonic_ns() // 1000
        bridge.tokens = 2.
        for index, delta_us in enumerate((-120_000, -20_000)):
            msg = Odometry()
            seconds, microseconds = divmod(now_us+delta_us, 1_000_000)
            msg.header.stamp.sec, msg.header.stamp.nanosec = seconds, microseconds*1000
            msg.header.frame_id, msg.child_frame_id = 'camera_init', 'aft_mapped'
            msg.pose.pose.orientation.w = 1.
            msg.pose.pose.position.x = index*1.2
            msg.twist.twist.linear.x = 12.
            bridge.on_odom(msg)
        h.spin(0.1)
        assert len(received) == 2  # 1.2 m / 0.1 s is valid at 12 m/s.
        status = VehicleStatus()
        status.timestamp, status.system_id, status.component_id = now_us, 1, 1
        status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        bridge.safety.on_status(status)
        msg.header.stamp.nanosec += 5_000_000
        if msg.header.stamp.nanosec >= 1_000_000_000:
            msg.header.stamp.sec += 1
            msg.header.stamp.nanosec -= 1_000_000_000
        msg.pose.pose.position.x += 3.
        bridge.on_odom(msg)
        h.spin(0.1)
        assert bridge.safety.policy.latched
        assert len(received) == 2  # Fault sample was blocked before publication.
    finally:
        bridge.destroy_node()


def test_missing_baro_confirmation_times_out_without_claiming_success(harness):
    h = harness
    assert not h.frame(2.04, position=5.04)
    h.spin()
    assert not h.commands
    h.frame(5.1)
    h.spin()
    assert h.commands  # Best-effort Land after a bounded transition wait.
    assert h.guard.snapshot['baro_transition_timed_out']
    assert not h.guard.snapshot['baro_transition_confirmed']
    assert h.guard.snapshot['baro_phase'] == 'ACTIVATION_TIMEOUT'
