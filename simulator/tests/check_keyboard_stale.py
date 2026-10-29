#!/usr/bin/env python3
"""A lost Windows key-state stream must release all manual commands."""
import ast
import base64
import os
from pathlib import Path
import subprocess
import time
from unittest.mock import patch

source=Path(__file__).resolve().parents[2]/'src/zav_control/zav_control/zav_flight_manager.py'
tree=ast.parse(source.read_text())
reader=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Win32MultiKeyReader')
namespace=dict(os=os,subprocess=subprocess,base64=base64,time=time)
exec(compile(ast.Module(body=[reader],type_ignores=[]),str(source),'exec'),namespace)
with patch.dict(os.environ,{'ZAV_SIM_PLATFORM':'native'}):
    node=namespace['Win32MultiKeyReader']()
r,w=os.pipe();os.set_blocking(r,False);node._pipe_fd=r
try:
    with patch.object(time,'monotonic',return_value=10.):
        os.write(w,b'1\n2\n65\n')
        assert node.poll_latest()==65 and node.active
    with patch.object(time,'monotonic',return_value=10.1):
        assert node.poll_latest()==65
    with patch.object(time,'monotonic',return_value=10.3):
        assert node.poll_latest()==0 and not node.active
    with patch.object(time,'monotonic',return_value=10.4):
        os.write(w,b'1\n');assert node.poll_latest()==1 and node.active
        os.close(w);w=None
        assert node.poll_latest()==0 and not node.active
    os.close(r);r=None
    assert node.poll_latest()==0  # a broken descriptor must also release input
finally:
    if r is not None:os.close(r)
    if w is not None:os.close(w)
print('KEYBOARD_STALE_OK: fresh input, newest mask, 250ms timeout, EOF, broken pipe')
