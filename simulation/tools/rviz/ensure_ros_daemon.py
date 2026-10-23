"""Start the ROS CLI daemon without probing an unbound WSL loopback port."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from ros2cli.daemon import make_xmlrpc_server
import rclpy

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--restart', action='store_true', help='Refresh this user/domain daemon after a simulator restart')
args = parser.parse_args()
if args.restart:
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            argv = proc.joinpath('cmdline').read_bytes().decode().split('\0')
            if not any('from ros2cli.daemon' in value for value in argv):
                continue
            if '--ros-domain-id' not in argv or argv[argv.index('--ros-domain-id') + 1] != domain:
                continue
            pid = int(proc.name)
            os.kill(pid, signal.SIGTERM)
            deadline = time.monotonic() + 1
            while proc.exists() and time.monotonic() < deadline:
                time.sleep(.05)
            if proc.exists():
                os.kill(pid, signal.SIGKILL)
        except (FileNotFoundError, ProcessLookupError):
            pass
# Probe by binding, never connecting to an unbound WSL localhost socket.
try:
    server = make_xmlrpc_server()
except OSError as error:
    import errno
    if error.errno == errno.EADDRINUSE:
        raise SystemExit(0)
    raise
server.server_close()
# ROS Humble's POSIX spawn_daemon shares the launching terminal's session.
# Detach explicitly so exiting that shell does not kill the CLI daemon.
log_path = Path('/tmp') / f"pegasus-ros2-daemon-{os.getuid()}-{os.environ.get('ROS_DOMAIN_ID', '0')}.log"
with log_path.open('a') as log:
    child = subprocess.Popen(
        [sys.executable, '-c', 'from ros2cli.daemon import main; main()',
         '--ros-domain-id', os.environ.get('ROS_DOMAIN_ID', '0'),
         '--rmw-implementation', rclpy.get_rmw_implementation_identifier()],
        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    time.sleep(.5)
    if child.poll() is not None:
        raise SystemExit(f'ROS daemon exited; see {log_path}')
