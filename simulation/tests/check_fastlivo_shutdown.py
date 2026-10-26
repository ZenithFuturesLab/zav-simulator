#!/usr/bin/env python3
"""Real terminal Ctrl+C, stuck ROS descendants, released listeners, and restart.

Uses isolated dummy children and ephemeral ports; never stops a user's ROS nodes.
"""
import json
import os
from pathlib import Path
import pty
import select
import shutil
import signal
import socket
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]

def port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]

with tempfile.TemporaryDirectory(prefix='zav-shutdown-') as temp:
    temp = Path(temp)
    rviz = temp / 'tools/rviz'; runtime = temp / 'tools/runtime'
    binaries = temp / 'bin'; workspace = temp / 'workspace'
    for directory in (rviz, runtime, binaries, workspace / 'install'):
        directory.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / 'tools/rviz/run_fastlivo.sh', rviz / 'run_fastlivo.sh')
    shutil.copy2(ROOT / 'tools/runtime/fastlivo_session.py', runtime / 'fastlivo_session.py')
    (rviz / 'ros_env.sh').write_text('export ZAV_WS_DIR="$TEST_WORKSPACE"\nexport ZAV_ROS_TRANSPORT=cyclone_tcp\n')
    (workspace / 'install/local_setup.bash').write_text('# fixture\n')
    payload = temp / 'payload.py'
    payload.write_text('''import os,sys,socket,signal,time,json,subprocess
from pathlib import Path
role=sys.argv[1]
signal.signal(signal.SIGINT,signal.SIG_IGN)
signal.signal(signal.SIGTERM,signal.SIG_IGN)
if role=='launch':
 for child in ('mapper','bridge'):
  subprocess.Popen([sys.executable,__file__,child],start_new_session=True)
 if os.environ.get('TEST_EARLY_EXIT')=='1':sys.exit(0)
else:
 s=socket.socket();s.bind(('127.0.0.1',int(os.environ['TEST_PORT_'+role.upper()])));s.listen(4)
with open(os.environ['TEST_EVENTS'],'a') as f:
 f.write(json.dumps({'pid':os.getpid(),'role':role})+'\\n');f.flush()
while True:time.sleep(10)
''')
    (runtime / 'sensor_relay.py').write_text('import runpy,sys\nsys.argv=[sys.argv[0],"relay"]\nrunpy.run_path('+repr(str(payload))+',run_name="__main__")\n')
    ros2 = binaries / 'ros2'
    ros2.write_text('#!/usr/bin/env python3\nimport runpy,sys\nsys.argv=[sys.argv[0],"launch"]\nrunpy.run_path('+repr(str(payload))+',run_name="__main__")\n')
    ros2.chmod(0o755)
    transcript = bytearray()
    ports = []
    while len(ports) < 3:
        number = port()
        if number not in ports: ports.append(number)
    for cycle, how in enumerate(('ctrl-c','ctrl-c','hup','term','launch-exit')):
        events = temp / f'events-{cycle}.jsonl'
        env = dict(os.environ, TEST_WORKSPACE=str(workspace), TEST_EVENTS=str(events),
                   TEST_PORT_RELAY=str(ports[0]), TEST_PORT_MAPPER=str(ports[1]),
                   TEST_PORT_BRIDGE=str(ports[2]),
                   PATH=str(binaries)+':'+os.environ['PATH'], PYTHONNOUSERSITE='1')
        # Isolate the fixed launch lock as well as the network ports.
        supervisor = runtime / 'fastlivo_session.py'
        source = (ROOT / 'tools/runtime/fastlivo_session.py').read_text()
        source = source.replace("f'/tmp/zav-fastlivo-{os.getuid()}.lock'", repr(str(temp / 'session.lock')))
        supervisor.write_text(source)
        if how=='launch-exit': env['TEST_EARLY_EXIT']='1'
        pid, master = pty.fork()
        if pid == 0:
            os.execve('/bin/bash', ['bash', str(rviz / 'run_fastlivo.sh')], env)
        def drain():
            if select.select([master], [], [], .02)[0]:
                try: transcript.extend(os.read(master, 65536))
                except OSError: pass
        try:
            if how!='launch-exit':
                deadline = time.monotonic()+8
                while time.monotonic()<deadline:
                    drain()
                    records=[json.loads(s) for s in events.read_text().splitlines()] if events.exists() else []
                    if len(records)==4: break
                else: raise AssertionError(('startup',transcript.decode(errors='replace')))
                duplicate=subprocess.run(['bash', str(rviz / 'run_fastlivo.sh')],env=env,
                                         capture_output=True,text=True,timeout=5)
                assert duplicate.returncode==1 and 'already running' in duplicate.stderr,duplicate
                if how=='ctrl-c':os.write(master,b'\x03')
                else:os.kill(pid,signal.SIGHUP if how=='hup' else signal.SIGTERM)
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                drain()
                done,status=os.waitpid(pid,os.WNOHANG)
                if done:break
            else:raise AssertionError(('shutdown timeout',transcript.decode(errors='replace')))
            expected={'ctrl-c':130,'hup':129,'term':143,'launch-exit':0}[how]
            assert os.waitstatus_to_exitcode(status)==expected,(status,transcript.decode(errors='replace'))
            records=[json.loads(s) for s in events.read_text().splitlines()]
            for record in records:
                assert not Path('/proc',str(record['pid'])).exists(),record
            for number in ports:
                with socket.socket() as sock:sock.bind(('127.0.0.1',number))
            print(f'PASS {how}: descendants reaped, all 3 TCP listeners released, lock reusable')
        finally:
            os.close(master)
            # Only test fixture processes, if an assertion fails.
            if events.exists():
                for line in events.read_text().splitlines():
                    try:os.kill(json.loads(line)['pid'],signal.SIGKILL)
                    except ProcessLookupError:pass
            try:os.kill(pid,signal.SIGKILL)
            except ProcessLookupError:pass
print('FASTLIVO_SHUTDOWN_OK: terminal Ctrl+C twice, HUP, TERM, launch failure/exit, duplicate rejection')
