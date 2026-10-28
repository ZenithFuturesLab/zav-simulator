"""Motion-only policy tests: slow/missing streams never imply a pose jump."""

import math
import pytest

from zav_control.lio_safety import LioSafety, SafetyConfig


def ready():
    safety = LioSafety()
    assert safety.odometry(10., (0, 0, 0), (1, 0, 0, 0), (1, 0, 0), 0.)
    safety.set_armed(True)
    return safety


def test_constant_speed_and_quaternion_sign():
    safety = ready()
    for tick in range(1, 201):
        dt = tick * .02
        q = (-1, 0, 0, 0) if tick % 2 else (1, 0, 0, 0)
        assert safety.odometry(10+dt, (dt, 0, 0), q, (1, 0, 0), tick * 2.)
        assert safety.evaluate()['ready'] and not safety.latched


def test_position_jump_is_immediate_and_never_rebases():
    safety = ready()
    previous = safety.last_odom
    assert not safety.odometry(10.02, (3.02, 0, 0), (1, 0, 0, 0), (1, 0, 0))
    assert safety.latched and safety.latch_reasons == ['position_residual']
    assert safety.last_odom == previous
    assert not safety.odometry(10.04, (.04, 0, 0), (1, 0, 0, 0), (1, 0, 0))
    safety.set_armed(False)
    assert safety.evaluate()['latched']


def test_attitude_jump_is_immediate():
    safety = ready()
    q = (math.sqrt(.5), 0, 0, math.sqrt(.5))
    assert not safety.odometry(10.02, (.02, 0, 0), q, (1, 0, 0))
    assert safety.latched and 'attitude_residual' in safety.latch_reasons


def test_small_correction_is_tolerated():
    safety = ready()
    assert safety.odometry(10.02, (.32, 0, 0), (1, 0, 0, 0), (1, 0, 0))
    assert not safety.latched


def test_normal_rotation_is_tolerated():
    safety = ready()
    angle = .02
    assert safety.odometry(10.02, (.02, 0, 0),
                           (math.cos(angle/2), 0, 0, math.sin(angle/2)), (1, 0, 0))
    assert not safety.latched


def test_measurement_gap_rebaselines_without_landing():
    safety = ready()
    assert safety.odometry(11., (100, 0, 0), (1, 0, 0, 0), (1, 0, 0), 50.)
    assert safety.evaluate()['comparison'] == 'gap_rebaseline'
    assert not safety.latched
    assert safety.odometry(11.02, (100.02, 0, 0), (1, 0, 0, 0), (1, 0, 0), 52.)
    assert safety.evaluate()['comparison'] == 'continuous'


def test_complete_silence_does_not_trigger_landing():
    safety = ready()
    for now in (1., 10., 100., 1000.):
        assert not safety.evaluate(now)['latched']


def test_frozen_and_backward_timestamps_are_dropped_without_landing():
    safety = ready()
    previous = safety.last_odom
    for stamp in (10., 9., 10.):
        assert not safety.odometry(stamp, (100, 0, 0), (1, 0, 0, 0), (1, 0, 0), 100.)
        assert not safety.latched and safety.last_odom == previous
    assert safety.odometry(10.02, (.02, 0, 0), (1, 0, 0, 0), (1, 0, 0), 200.)


@pytest.mark.parametrize('position,quaternion,velocity', [
    ((math.nan, 0, 0), (1, 0, 0, 0), (1, 0, 0)),
    ((0, 0, 0), (0, 0, 0, 0), (1, 0, 0)),
    ((0, 0, 0), (math.inf, 0, 0, 0), (1, 0, 0)),
    ((0, 0, 0), (1, 0, 0, 0), (math.nan, 0, 0)),
])
def test_invalid_sample_drops_without_landing(position, quaternion, velocity):
    safety = ready()
    assert not safety.odometry(10.02, position, quaternion, velocity)
    assert not safety.latched
    assert safety.evaluate()['comparison'] == 'invalid_sample_dropped'
    assert safety.odometry(10.04, (.04, 0, 0), (1, 0, 0, 0), (1, 0, 0))


def test_ground_jump_never_latches():
    safety = LioSafety()
    safety.odometry(10., (0, 0, 0), (1, 0, 0, 0), (1, 0, 0))
    assert not safety.odometry(10.02, (100, 0, 0), (1, 0, 0, 0), (1, 0, 0))
    assert not safety.latched
    assert safety.odometry(10.04, (100.02, 0, 0), (1, 0, 0, 0), (1, 0, 0))


def test_child_velocity_rotation_and_high_speed_motion():
    safety = LioSafety()
    q = (math.sqrt(.5), 0, 0, math.sqrt(.5))
    safety.odometry(10., (0, 0, 0), q, (12, 0, 0))
    safety.set_armed(True)
    assert safety.odometry(10.1, (0, 1.2, 0), q, (12, 0, 0))
    assert safety.metrics['position_residual_m'] < 1e-12


@pytest.mark.parametrize('kwargs', [{'position_slack': -1}, {'prediction_horizon': math.nan},
                                   {'angle_slack': 0}])
def test_bad_config(kwargs):
    with pytest.raises(ValueError):
        SafetyConfig(**kwargs)
