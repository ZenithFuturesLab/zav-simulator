"""Verify the named parameter protocol without opening any real MAVLink link."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

spec = importlib.util.spec_from_file_location('baro_parameter',
    Path(__file__).resolve().parents[3] / 'simulator/tools/runtime/baro_parameter_bridge.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def message(kind, *, value=0, system=1, component=1, name=m.PARAMETER, typ=m.INT32):
    return SimpleNamespace(get_type=lambda: kind, get_srcSystem=lambda: system,
                           get_srcComponent=lambda: component, param_id=name,
                           param_type=typ, param_value=m.encode_int32(value))


def link():
    pending = []
    connection = SimpleNamespace(mav=SimpleNamespace(param_set_send=Mock(), param_request_read_send=Mock()),
                                 recv_match=lambda **kwargs: pending.pop(0) if pending else None)
    client = m.ParameterLink(connection)
    client.target = (1, 1)
    return client, connection, pending


def test_int32_uses_px4_bytewise_wire_encoding():
    for value in (0, 1):
        assert m.decode_int32(m.encode_int32(value)) == value
    assert m.encode_int32(1) != 1.0


def test_ack_value_and_type_are_verified_and_expire():
    client, connection, pending = link()
    pending.extend([message('HEARTBEAT'), message('PARAM_VALUE', value=0)])
    client.tick(1, True, 10.)
    args = connection.mav.param_set_send.call_args.args
    assert args[:3] == (1, 1, b'EKF2_BARO_CTRL')
    assert m.decode_int32(args[3]) == 1 and args[4] == m.INT32
    assert not client.snapshot(1, 10.)['confirmed']
    pending.append(message('PARAM_VALUE', value=1))
    client.tick(1, True, 10.1)
    assert client.snapshot(1, 10.1)['confirmed']
    assert not client.snapshot(1, 14.)['confirmed']


def test_disabling_while_armed_is_forbidden():
    client, connection, pending = link()
    pending.extend([message('HEARTBEAT'), message('PARAM_VALUE', value=1)])
    client.tick(0, True, 10.)
    connection.mav.param_set_send.assert_not_called()
    client.tick(0, False, 10.6)
    assert m.decode_int32(connection.mav.param_set_send.call_args.args[3]) == 0


def test_other_vehicle_wrong_parameter_and_wrong_type_do_not_confirm():
    client, connection, pending = link()
    pending.extend([message('HEARTBEAT'), message('PARAM_VALUE', value=1, system=2),
                    message('PARAM_VALUE', value=1, component=191),
                    message('PARAM_VALUE', value=1, name='EKF2_EV_CTRL'),
                    message('PARAM_VALUE', value=1, typ=9)])
    client.tick(1, True, 10.)
    assert client.value is None and not client.snapshot(1, 10.)['confirmed']


def test_missing_or_stale_heartbeat_prevents_writes():
    client, connection, pending = link()
    client.tick(1, True, 10.)
    connection.mav.param_set_send.assert_not_called()
    pending.append(message('HEARTBEAT'))
    client.poll(10.)
    client.tick(1, True, 14.)
    connection.mav.param_set_send.assert_not_called()
