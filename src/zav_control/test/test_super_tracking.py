"""Tracking conversion and fail-closed behavior; no aircraft publishers."""
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from zav_control.super_tracking import tracking_setpoint


def test_enu_error_and_velocity_to_ned_with_estimator_origin():
    p, v = tracking_setpoint([1.1, 2.2, 3.1], [1., 2., 3.], [10., 20., -4.], [.3, .4, .2])
    assert p == pytest.approx([10.2, 20.1, -4.1])
    assert v == pytest.approx([.4, .3, -.2])


@pytest.mark.parametrize('target,velocity', [([2., 0., 0.], [0., 0., 0.]),
    ([math.nan, 0., 0.], [0., 0., 0.]), ([0., 0., 0.], [2., 0., 0.])])
def test_rejects_divergence_nonfinite_and_overspeed(target, velocity):
    with pytest.raises(ValueError):
        tracking_setpoint(target, [0., 0., 0.], [0., 0., 0.], velocity)


def manager_fixture(monkeypatch):
    from zav_control import zav_flight_manager as module
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import TrajectorySetpoint
    monkeypatch.setattr(module, 'time', NS(monotonic=lambda: 10.0))
    cmd, odom = Odometry(), Odometry()
    for m in (cmd, odom):
        m.header.frame_id = 'camera_init'
        m.header.stamp.sec = 10
    for i in (0,7,14):
        odom.twist.covariance[i] = .04
    cmd.child_frame_id = 'camera_init'
    # Identity body attitude plus simulator IMU mounting pitch.
    odom.pose.pose.orientation.w = math.cos(math.radians(11))
    odom.pose.pose.orientation.y = math.sin(math.radians(11))
    cmd.pose.pose.position.x = .1
    manager = NS(super_blocked=False, super_command=cmd, super_odom=odom,
        super_command_rx=10., super_odom_rx=10., last_position_time=10.,
        position_ready=True, lio_safety_ready=True, lio_safety_rx=10.,
        current_x=5., current_y=6., current_alt=2., current_yaw=math.pi/2,
        hold_x=1., hold_y=2., hold_z=-2., hold_yaw=math.pi/2,
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)),
        get_logger=lambda: NS(error=Mock()), setpoint_pub=NS(publish=Mock()))
    return module, manager, TrajectorySetpoint()


def test_manager_tracks_and_holds_on_stale_then_requires_reactivation(monkeypatch):
    module, manager, sp = manager_fixture(monkeypatch)
    module.ZAVFlightManager._super_setpoint(manager, sp)
    assert list(sp.position) == pytest.approx([5., 6.1, -2.])
    assert not manager.super_blocked
    manager.current_x = 5.1
    manager.super_command_rx = 9.0
    module.ZAVFlightManager._super_setpoint(manager, sp)
    assert manager.super_blocked
    assert list(sp.position) == pytest.approx([5.1, 6., -2.])
    manager.super_command_rx = 10.
    manager.current_x = 7.
    module.ZAVFlightManager._super_setpoint(manager, sp)
    assert list(sp.position) == pytest.approx([5.1, 6., -2.])


@pytest.mark.parametrize('fault', ['frame', 'stamp', 'heading', 'safety'])
def test_manager_blocks_invalid_frame_epoch_heading_and_safety(monkeypatch, fault):
    module, manager, sp = manager_fixture(monkeypatch)
    if fault == 'frame':
        manager.super_command.header.frame_id = 'map'
    elif fault == 'stamp':
        manager.super_command.header.stamp.sec = 20
    elif fault == 'heading':
        manager.current_yaw = 0.
    else:
        manager.lio_safety_ready = False
    module.ZAVFlightManager._super_setpoint(manager, sp)
    assert manager.super_blocked
    assert list(sp.velocity) == [0., 0., 0.]


@pytest.mark.parametrize('field,reason', [
    ('super_command_rx', 'tracking command timeout'),
    ('super_odom_rx', 'LIO odometry timeout'),
    ('last_position_time', 'PX4 local position stale or invalid'),
])
def test_timeout_reports_which_stream_stopped(monkeypatch, field, reason):
    module, manager, sp = manager_fixture(monkeypatch)
    logger = NS(error=Mock())
    manager.get_logger = lambda: logger
    setattr(manager, field, 9.0)
    module.ZAVFlightManager._super_setpoint(manager, sp)
    assert manager.super_blocked
    assert reason in logger.error.call_args.args[0]
    assert list(sp.velocity) == [0., 0., 0.]


def test_delayed_position_is_compared_at_command_epoch():
    from zav_control.super_tracking import predict_position
    position = predict_position([0.,0.,0.], [1.,0.,0.], .2)
    p, _ = tracking_setpoint([.6,0.,0.], position, [5.,6.,0.], [1.,0.,0.])
    assert p == pytest.approx([5.,6.4,0.])
    with pytest.raises(ValueError):
        tracking_setpoint([.8,0.,0.], position, [5.,6.,0.], [1.,0.,0.])


@pytest.mark.parametrize('age', [.31, -0.06, math.nan])
def test_prediction_rejects_invalid_epochs(age):
    from zav_control.super_tracking import predict_position
    with pytest.raises(ValueError):
        predict_position([0.,0.,0.], [1.,0.,0.], age)


def test_execution_enable_requires_armed_active_unblocked_manager(monkeypatch):
    from zav_control import zav_flight_manager as module
    monkeypatch.setattr(module, 'time', NS(monotonic=lambda: 10.))
    manager=NS(super_tracking=True, flight_mode='MANUAL', setpoints_enabled=True,
               is_armed=True, is_flying=True, lio_safety_latched=False, super_blocked=False,
               execution_enable_pub=NS(publish=Mock()), execution_last_value=None,
               execution_last_publish=0.)
    announce=module.ZAVFlightManager._announce_execution
    announce(manager,force=True)
    assert not manager.execution_enable_pub.publish.call_args.args[0].data
    manager.flight_mode='OFFBOARD_TRAJ'
    announce(manager,force=True)
    assert manager.execution_enable_pub.publish.call_args.args[0].data
    for attr in ('super_blocked','lio_safety_latched'):
        setattr(manager,attr,True)
        announce(manager,force=True)
        assert not manager.execution_enable_pub.publish.call_args.args[0].data
        setattr(manager,attr,False)
    manager.setpoints_enabled=False
    announce(manager,force=True)
    assert not manager.execution_enable_pub.publish.call_args.args[0].data


def test_future_imu_feedback_selects_latest_real_measurement_before_command():
    from zav_control.super_tracking import feedback_at_epoch
    # An IMU callback is a full render frame ahead of the command/clock.
    assert feedback_at_epoch([(10., 'at_command'), (10.05, 'future')], 10.) == 'at_command'
    assert feedback_at_epoch([(9.99, 'older'), (10.01, 'future')], 10.) == 'older'
    with pytest.raises(ValueError):
        feedback_at_epoch([(10.01, 'future')], 10.)


def test_manager_uses_buffered_epoch_instead_of_future_imu(monkeypatch):
    from nav_msgs.msg import Odometry
    module, manager, sp = manager_fixture(monkeypatch)
    at_command=manager.super_odom
    future=Odometry()
    future.header.stamp.sec=10
    future.header.stamp.nanosec=60_000_000
    future.pose.pose.position.x=100.
    manager.super_odom=future
    manager.super_odom_history=[(10.,at_command),(10.06,future)]
    module.ZAVFlightManager._super_setpoint(manager,sp)
    assert not manager.super_blocked
    assert list(sp.position) == pytest.approx([5.,6.1,-2.])


def test_higher_speed_requires_explicit_bounded_sitl_limit():
    with pytest.raises(ValueError):
        tracking_setpoint([0,0,0],[0,0,0],[0,0,0],[2.,0,0])
    _, velocity=tracking_setpoint([0,0,0],[0,0,0],[0,0,0],[2.,0,0],max_speed=2.2)
    assert velocity == [0,2.,0]
    for limit in [0.,float('inf'),25.]:
        with pytest.raises(ValueError):
            tracking_setpoint([0,0,0],[0,0,0],[0,0,0],[2.,0,0],max_speed=limit)


def test_velocity_yaw_faces_enu_motion_with_bounded_ned_rotation():
    from zav_control.super_tracking import velocity_heading
    assert velocity_heading([1.,0.,0.],0.,.1)==pytest.approx(.1)
    assert velocity_heading([0.,1.,0.],.1,.2)==pytest.approx(0.)
    assert velocity_heading([0.,0.,1.],.7,.1)==pytest.approx(.7)
    target=-math.pi+.01
    result=velocity_heading([math.sin(target),math.cos(target),0.],math.pi-.01,.1)
    assert result==pytest.approx(target)
    with pytest.raises(ValueError):velocity_heading([1.,0.,0.],0.,math.nan)


def test_manager_opt_in_yaw_follows_flight_direction_without_position_changes(monkeypatch):
    module,manager,sp=manager_fixture(monkeypatch)
    manager.super_follow_velocity_yaw=True
    manager.super_yaw_wall=9.9
    manager.super_command.twist.twist.linear.y=.4
    module.ZAVFlightManager._super_setpoint(manager,sp)
    assert not manager.super_blocked
    assert sp.yaw==pytest.approx(math.pi/2-.1)
    assert list(sp.position)==pytest.approx([5.,6.1,-2.])


def test_acceleration_feedforward_excludes_gravity_and_converts_enu_to_ned():
    from zav_control.super_tracking import acceleration_feedforward
    assert acceleration_feedforward([.2,.3,.4])==pytest.approx([.3,.2,-.4])
    for acceleration in ([math.nan,0,0],[4.,0,0]):
        with pytest.raises(ValueError):acceleration_feedforward(acceleration)


def test_manager_uses_only_matching_acceleration_epoch(monkeypatch):
    from geometry_msgs.msg import AccelStamped
    module,manager,sp=manager_fixture(monkeypatch)
    acc=AccelStamped();acc.header.stamp.sec=10;acc.header.frame_id='camera_init'
    acc.accel.linear.x=.2;acc.accel.linear.y=.3;acc.accel.linear.z=.4
    manager.super_acceleration_feedforward=True
    manager.super_ff_last_match_wall=None
    manager.super_accel_samples={(10,0):(acc,10.)}
    module.ZAVFlightManager._super_setpoint(manager,sp)
    assert not manager.super_blocked and manager.super_acceleration_active
    assert list(sp.acceleration)==pytest.approx([.3,.2,-.4])
    manager.super_accel_samples={(11,0):(acc,10.)}
    manager.super_ff_last_match_wall=9.5
    module.ZAVFlightManager._super_setpoint(manager,sp)
    assert manager.super_blocked and not manager.super_acceleration_active
    assert all(math.isnan(v) for v in sp.acceleration)


def test_manager_blocks_invalid_acceleration_even_when_position_is_valid(monkeypatch):
    from geometry_msgs.msg import AccelStamped
    module,manager,sp=manager_fixture(monkeypatch)
    acc=AccelStamped();acc.header.stamp.sec=10;acc.header.frame_id='camera_init';acc.accel.linear.z=20.
    manager.super_acceleration_feedforward=True;manager.super_ff_last_match_wall=None
    manager.super_accel_samples={(10,0):(acc,10.)}
    module.ZAVFlightManager._super_setpoint(manager,sp)
    assert manager.super_blocked and not manager.super_acceleration_active
    assert all(math.isnan(v) for v in sp.acceleration)


def ommpc_fixture(monkeypatch):
    from px4_msgs.msg import VehicleRatesSetpoint
    module,manager,sp=manager_fixture(monkeypatch)
    rates=VehicleRatesSetpoint();rates.timestamp=10_000_000
    rates.roll=.1;rates.pitch=-.2;rates.yaw=.3;rates.thrust_body=[0.,0.,-.41]
    manager.super_controller='ommpc';manager.ommpc_active=False
    manager.ommpc_rates=rates;manager.ommpc_rates_rx=10.;manager.ommpc_request_wall=9.9
    manager.ommpc_yaw_pub=NS(publish=Mock());manager.ommpc_rates_pub=NS(publish=Mock())
    manager.publish_offboard_mode=Mock()
    return module,manager,sp


def test_ommpc_handoff_publishes_real_float_rates_without_position_commands(monkeypatch):
    module,manager,sp=ommpc_fixture(monkeypatch)
    module.ZAVFlightManager._super_setpoint(manager,sp)
    assert manager.ommpc_active and not manager.super_blocked
    assert manager.setpoint_pub.publish.call_count==0
    output=manager.ommpc_rates_pub.publish.call_args[0][0]
    assert list(output.thrust_body)==pytest.approx([0.,0.,-.41])
    assert output.reset_integral
    assert output.timestamp==0 and manager.ommpc_rates.timestamp==10_000_000


@pytest.mark.parametrize('fault',['stale','nan','positive_thrust','rate_bound'])
def test_ommpc_invalid_output_reverts_to_position_hold_and_latches(monkeypatch,fault):
    module,manager,sp=ommpc_fixture(monkeypatch)
    manager.ommpc_active=True
    if fault=='stale':manager.ommpc_request_wall=9.;manager.ommpc_rates_rx=9.
    elif fault=='nan':manager.ommpc_rates.roll=math.nan
    elif fault=='positive_thrust':manager.ommpc_rates.thrust_body=[0.,0.,.4]
    else:manager.ommpc_rates.pitch=3.
    module.ZAVFlightManager._super_setpoint(manager,sp)
    assert manager.super_blocked and not manager.ommpc_active
    assert manager.ommpc_rates_pub.publish.call_count==0
    assert list(sp.velocity)==[0.,0.,0.]


def test_ommpc_offboard_priority_is_body_rates_only_during_tracking():
    from zav_control.zav_flight_manager import ZAVFlightManager
    manager=NS(ommpc_active=True,flight_mode='OFFBOARD_TRAJ',setpoints_enabled=True,
               super_blocked=False,offboard_mode_pub=NS(publish=Mock()))
    ZAVFlightManager.publish_offboard_mode(manager)
    mode=manager.offboard_mode_pub.publish.call_args[0][0]
    assert mode.body_rate and not any((mode.position,mode.velocity,mode.acceleration,mode.attitude))
    manager.flight_mode='MANUAL'
    ZAVFlightManager.publish_offboard_mode(manager)
    mode=manager.offboard_mode_pub.publish.call_args[0][0]
    assert mode.position and not mode.body_rate
