"""Invalid EKF velocity must block takeoff before any arm/mode command."""
import ast
import math
from pathlib import Path
import time
from types import SimpleNamespace
source = Path(__file__).resolve().parents[2] / 'src/zav_control/zav_control/zav_flight_manager.py'
tree = ast.parse(source.read_text())
method = next(n for c in tree.body if isinstance(c, ast.ClassDef) and c.name == 'ZAVFlightManager'
              for n in c.body if isinstance(n, ast.FunctionDef) and n.name == 'update_teleop')
namespace = dict(math=math, time=time)
exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
for velocity in (math.inf, math.nan):
    commands = []
    node = SimpleNamespace(lio_safety_latched=False, lio_safety_ready=True,
                           lio_safety_rx=time.monotonic(), position_ready=True,
                           last_position_time=time.monotonic(), current_speed=velocity,
                           send_vehicle_command=lambda *a: commands.append(a))
    assert namespace['update_teleop'](node, 'T') is True
    assert commands == []
print('PASS: invalid EKF velocity blocks takeoff even with valid position/safety')
