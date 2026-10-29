#!/usr/bin/env python3
"""Exercise actual session argument routing without starting ROS or aircraft nodes."""
import importlib.util
from pathlib import Path
import os
import sys
from unittest.mock import patch

root = Path(__file__).resolve().parents[2]
path = root / 'simulator/tools/runtime/fastlivo_session.py'
spec = importlib.util.spec_from_file_location('lio_session_test', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
calls = []

def capture(relay, launch, lock, *args):
    calls.append((relay, launch, lock))
    return 0

for backend in ('fastlivo', 'super_lio'):
    argv = [str(path), '--rviz-dir', str(root/'simulator/tools/rviz'),
            '--backend', backend, 'rviz:=False']
    with patch.object(module, 'run_session', capture), patch.object(sys, 'argv', argv), \
            patch.dict(os.environ, {'ZAV_WS_DIR':str(root)}):
        assert module.main() == 0
fast, super_lio = calls
assert fast[2] == super_lio[2], 'Mappers must share the same exclusive lock'
assert fast[0][fast[0].index('--camera-port')+1] == '17443'
assert super_lio[0][super_lio[0].index('--camera-port')+1] == '0'
assert 'backend:=super_lio' in super_lio[1]
assert 'rviz:=False' in fast[1] and 'rviz:=False' in super_lio[1]
print('LIO_BACKEND_OK: shared lock, backend routing, Super-LIO skips camera relay')
