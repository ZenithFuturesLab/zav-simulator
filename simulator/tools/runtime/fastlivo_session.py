"""Own FAST-LIVO's process tree and reap it before releasing its launch lock.

Linux/WSL only, like the ROS side of the simulator. Isaac itself is not stopped.
"""
import argparse
import ctypes
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


def children_of(parent, recursive=True):
    """Include descendants that ROS launch moved into another process group."""
    parents = {}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            # comm can contain spaces/parentheses; fields after its last ')' are stable.
            fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
            parents[int(proc.name)] = int(fields[1])
        except (OSError, IndexError, ValueError):
            continue
    if not recursive:
        return {pid for pid, ppid in parents.items() if ppid == parent}
    owned = {parent}
    while True:
        added = {pid for pid, ppid in parents.items() if ppid in owned} - owned
        if not added:
            return owned - {parent}
        owned.update(added)


def reap():
    while True:
        try:
            if os.waitpid(-1, os.WNOHANG)[0] == 0:
                return
        except ChildProcessError:
            return


def signal_children(signum, sent):
    # PR_SET_CHILD_SUBREAPER keeps orphaned grandchildren beneath this supervisor.
    # Address actual PIDs, not an assumed setsid PID/process-group relationship.
    # ROS launch forwards SIGINT itself. Avoid interrupting each ROS node twice
    # during graceful shutdown; escalation still reaches the complete tree.
    recursive = signum != signal.SIGINT
    for pid in children_of(os.getpid(), recursive=recursive) - sent:
        try:
            os.kill(pid, signum)
            sent.add(pid)
        except ProcessLookupError:
            pass


def shutdown():
    print('[FAST-LIVO] Stopping mapper, bridge and owned relay...', flush=True)
    for signum, grace in ((signal.SIGINT, 2.0), (signal.SIGTERM, 1.0),
                          (signal.SIGKILL, 2.0)):
        deadline = time.monotonic() + grace
        sent = set()
        while True:
            signal_children(signum, sent)
            reap()
            if not children_of(os.getpid()):
                print('[FAST-LIVO] All owned processes stopped; TCP listeners released.', flush=True)
                return True
            if time.monotonic() >= deadline:
                break
            time.sleep(.05)
    print('[FAST-LIVO] ERROR: processes remain: ' + str(sorted(children_of(os.getpid()))),
          file=sys.stderr, flush=True)
    return False


def run_session(relay_command, launch_command, lock_path, relay_env=None, log_path=None,
                preflight=None):
    """Commands are separate argv lists; injectable for real process/PTY tests."""
    lock_path = Path(lock_path)
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    # Keep this inode: unlinking a lock file lets two launchers lock different inodes.
    with lock_path.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('FAST-LIVO is already running; stop its launcher with Ctrl+C first.',
                  file=sys.stderr, flush=True)
            return 1
        if preflight:
            preflight()
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(36, 1, 0, 0, 0) != 0:  # PR_SET_CHILD_SUBREAPER
            raise OSError(ctypes.get_errno(), 'Cannot supervise ROS descendants')
        stopped = threading.Event()
        exit_code = [0]
        def stop(signum, _frame):
            exit_code[0] = 128 + signum
            stopped.set()
        previous = {s: signal.signal(s, stop) for s in
                    (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
        log = None
        try:
            if log_path:
                log = open(log_path, 'w')
            relay = subprocess.Popen(relay_command, env=relay_env, start_new_session=True,
                                     stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                     close_fds=True)
            launch = subprocess.Popen(launch_command, start_new_session=True,
                                      stdin=subprocess.DEVNULL, close_fds=True)
            while not stopped.wait(.1):
                code = launch.poll()
                if code is not None:
                    exit_code[0] = code if code >= 0 else 128 - code
                    break
                # Exit 0 means the RViz-owned singleton relay was already present.
                if relay.poll() not in (None, 0):
                    print('[FAST-LIVO] Sensor relay failed; stopping this session.',
                          file=sys.stderr, flush=True)
                    exit_code[0] = 1
                    break
        finally:
            if not shutdown():
                exit_code[0] = 1
            if log:
                log.close()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
        return exit_code[0]


def check_existing_nodes(workspace):
    """The old shell's lock may be gone even though its ROS nodes survived."""
    executables = {str(workspace / 'install/fast_livo/lib/fast_livo/fastlivo_mapping'),
                   str(workspace / 'install/zav_control/lib/zav_control/lio_px4_bridge')}
    stale = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            args = (proc / 'cmdline').read_bytes().decode().split('\0')
            if executables.intersection(args):
                stale.append(int(proc.name))
        except (OSError, UnicodeError):
            continue
    if stale:
        raise RuntimeError(f'Existing FAST-LIVO/bridge processes: {sorted(stale)}. '
                           'Stop them before restarting; refusing duplicate publishers.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rviz-dir', required=True, type=Path)
    args, launch_args = parser.parse_known_args()
    directory = args.rviz_dir.resolve()
    relay_env = dict(os.environ)
    if os.environ.get('ZAV_ROS_TRANSPORT') == 'cyclone_tcp':
        relay_env['CYCLONEDDS_URI'] = (directory / 'cyclonedds_publisher.xml').as_uri()
    command = [sys.executable, str(directory.parent / 'runtime/sensor_relay.py'), 'client',
               '--host', os.environ.get('ZAV_SENSOR_HOST', '127.0.0.1'),
               '--camera-port', '17443', '--imu-port', '17444', '--livox-port', '17445']
    try:
        return run_session(command, ['ros2', 'launch', str(directory / 'fastlivo_sim.launch.py'),
                                     *launch_args],
                           f'/tmp/zav-fastlivo-{os.getuid()}.lock', relay_env,
                           '/tmp/zav-fastlivo-relay.log',
                           lambda: check_existing_nodes(Path(os.environ['ZAV_WS_DIR'])))
    except (OSError, RuntimeError) as exc:
        print(f'[FAST-LIVO] {exc}', file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
