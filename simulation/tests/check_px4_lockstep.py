#!/usr/bin/env python3
"""Exercise the actual MAVLink receive method without Kit or a live vehicle."""
import ast
from pathlib import Path
from types import SimpleNamespace

source = Path(__file__).resolve().parents[1] / 'pegasus/overlay/logic/backends/px4_mavlink_backend.py'
tree = ast.parse(source.read_text())
method = next(n for c in tree.body if isinstance(c, ast.ClassDef) and c.name == 'PX4MavlinkBackend'
              for n in c.body if isinstance(n, ast.FunctionDef) and n.name == 'poll_mavlink_messages')
clock = SimpleNamespace(now=0.0)
ns = {'time': SimpleNamespace(monotonic=lambda: clock.now),
      'mavutil': SimpleNamespace(mavlink=SimpleNamespace(MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS=93))}
exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), ns)

class Connection:
    def __init__(self, messages): self.messages = list(messages); self.calls = []
    def recv_match(self, **kwargs):
        self.calls.append(kwargs)
        if self.messages:
            clock.now += .01
            return self.messages.pop(0)
        if kwargs['blocking']: clock.now += kwargs['timeout']
        return None

def vehicle(messages, first=True):
    clock.now = 0.0
    handled = []
    node = SimpleNamespace(_received_first_hearbeat=True, _received_first_actuator=first,
                           _enable_lockstep=True, _connection=Connection(messages),
                           handle_control=lambda *args: handled.append(args))
    return node, handled

actuator = SimpleNamespace(id=93, time_usec=100, controls=[0.1]*4, mode=0, flags=0)
node, handled = vehicle([actuator]); ns['poll_mavlink_messages'](node)
assert len(handled) == 1 and node._received_actuator and len(node._connection.calls) == 1
node, handled = vehicle([]); ns['poll_mavlink_messages'](node)
assert not handled and clock.now <= .200001 and len(node._connection.calls) == 1
node, handled = vehicle([SimpleNamespace(id=0)]*100); ns['poll_mavlink_messages'](node)
assert not handled and clock.now <= .210001 and len(node._connection.calls) < 30
node, handled = vehicle([], first=False); ns['poll_mavlink_messages'](node)
assert not node._connection.calls[0]['blocking'] and clock.now == 0
print('LOCKSTEP_OK: actuator delivery, bounded disarm/disconnect wait, non-actuator traffic, startup')
