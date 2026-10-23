# [ZAV 변경] 이전 실행기의 gateway 자식까지 정리해 중복 포트와 지연 EXIT trap을 방지합니다.
"""Remove legacy launchers/gateways before the singleton simulator starts."""
import os
from pathlib import Path
import signal

workspace = Path(os.environ.get('ZAV_WS_DIR', Path.home() / 'zenith_ws'))
runner = str(workspace / 'simulation/scripts/run_zav_sim.sh')
runners = {runner, str(Path.home() / 'run_zav_sim.sh')}
# [ZAV 변경] 이전 diagnostics 경로와 배포된 runtime 경로를 모두 정리합니다.
pegasus = Path(os.environ.get('PEGASUS_DIR', Path.home() / 'zenith_ws/src/PegasusSimulator'))
gateways = {str(pegasus / 'tools' / sub / 'px4_dds_gateway.py') for sub in ('runtime', 'diagnostics')}
simulation = str(pegasus / 'examples/zav_sim.py')
records = {}
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():
        continue
    try:
        if proc.stat().st_uid != os.getuid():
            continue
        argv = proc.joinpath('cmdline').read_bytes().decode().split('\0')
        status = proc.joinpath('status').read_text().splitlines()
        parent = int(next(s for s in status if s.startswith('PPid:')).split()[1])
        records[int(proc.name)] = (parent, argv)
    except (FileNotFoundError, ProcessLookupError):
        pass
# The invoking runner is our parent. Exact argv matching avoids killing unrelated shells.
stale = {pid for pid, (ppid, argv) in records.items()
         if (runners.intersection(argv) and pid != os.getppid()) or gateways.intersection(argv) or simulation in argv}
# Gateways have multiprocessing children whose argv does not include the filename.
while True:
    children = {pid for pid, (ppid, argv) in records.items() if ppid in stale}
    added = children - stale
    if not added:
        break
    stale.update(added)
# Kill launcher parents first so their old global EXIT trap cannot kill the new run.
for pid in sorted(stale, key=lambda pid: not runners.intersection(records[pid][1])):
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
