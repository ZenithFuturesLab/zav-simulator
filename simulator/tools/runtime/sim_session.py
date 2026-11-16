"""Small, validated rendezvous record for the single local simulator session.

The record contains configuration only. It lets navigation launched from a
second terminal use the same ROS domain, even when shell startup files differ.
"""
import argparse
import json
import os
from pathlib import Path
import tempfile
import time

WORKSPACE = Path(__file__).resolve().parents[3]
RUNNER = WORKSPACE / 'simulator/scripts/run_zav_sim.sh'


def session_path():
    return Path(f'/tmp/zav-sim-session-{os.getuid()}.json')


def read_session(path=None):
    path = Path(path) if path else session_path()
    try:
        if path.stat().st_uid != os.getuid():
            return None
        data = json.loads(path.read_text())
        pid = int(data['pid'])
        domain = int(data['domain'])
        if pid <= 1 or not 0 <= domain <= 232:
            return None
        proc = Path('/proc') / str(pid)
        if proc.stat().st_uid != os.getuid():
            return None
        argv = (proc / 'cmdline').read_bytes().split(b'\0')
        if os.fsencode(RUNNER) not in argv:
            return None
        if data['scene'] not in ('warehouse', 'high_speed', 'dense', 'mixed'):
            return None
        data['domain'] = domain
        return data
    except (OSError, ValueError, TypeError, KeyError):
        return None


def write_session(pid):
    data = {'pid': int(pid), 'domain': int(os.environ.get('ROS_DOMAIN_ID', '0')),
            'scene': os.environ.get('ZAV_SUPER_SCENE') or 'warehouse',
            'transport': os.environ.get('ZAV_ROS_TRANSPORT', 'fastdds'),
            'localhost_only': os.environ.get('ROS_LOCALHOST_ONLY', '0')}
    path = session_path()
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent,
                                     prefix=f'.zav-sim-session-{os.getuid()}-', delete=False) as f:
        json.dump(data, f)
        temporary = Path(f.name)
    temporary.replace(path)


def clear_session(pid):
    path = session_path()
    try:
        if path.stat().st_uid == os.getuid() and json.loads(path.read_text()).get('pid') == int(pid):
            path.unlink()
    except (OSError, ValueError, TypeError):
        pass


def wait_for_session(scenario, timeout=30.):
    deadline = time.monotonic() + timeout
    while True:
        session = read_session()
        if session:
            if session['scene'] != scenario:
                raise RuntimeError(f"Simulator scene is {session['scene']}, requested {scenario}. "
                                   f"Restart with ./run_super_sim.sh {scenario}")
            return session
        if time.monotonic() >= deadline:
            raise RuntimeError(f'No running simulator session. Start ./run_super_sim.sh {scenario} first.')
        time.sleep(.2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('write', 'clear'))
    parser.add_argument('--pid', type=int, required=True)
    args = parser.parse_args()
    (write_session if args.action == 'write' else clear_session)(args.pid)
