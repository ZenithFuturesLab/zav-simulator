"""Installation regression: fresh target, repeat deploy, asset location and accidental old overwrite."""
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import sys

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('deploy', root / 'deploy_sim.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
with tempfile.TemporaryDirectory(prefix='zav-deploy-') as temp:
    temp = Path(temp)
    pegasus, home = temp / 'Pegasus with spaces', temp / 'home'
    sim = pegasus / module.SIM_ROOT
    sim.mkdir(parents=True)
    (sim / 'params.py').write_text('ROBOTS_ASSETS = "assets/Robots"\nROBOTS = {}\nWORLD_SETTINGS = {"rendering_dt": 1/60}\n')
    cmd = [sys.executable, str(root / 'deploy_sim.py'), '--pegasus-dir', str(pegasus), '--home-dir', str(home)]
    subprocess.run(cmd, check=True, stdout=subprocess.PIPE)
    subprocess.run(cmd, check=True, stdout=subprocess.PIPE)
    subprocess.run(cmd + ['--check'], check=True, stdout=subprocess.PIPE)
    assert (sim / 'assets/Robots/ZAV_v1/ZAV_v1.usd').is_file()
    assert (sim / 'assets/Sensors/Mid-360s.usda').is_file()
    assert not (sim / 'assets/ZAV_v1').exists()
    assert '1/60' in (sim / 'params.py').read_text()
    # setup re-run must copy the current sensor-aware source, not the old plane-only launcher.
    assert 'lidar_config=MID360_CONFIG' in (pegasus / 'examples/zav_sim.py').read_text()
    assert (pegasus / 'examples/slam_test_scene.py').is_file()
    assert (pegasus / 'tools/runtime/sensor_relay.py').is_file()
    assert 'ZAV_v1' in (sim / 'logic/vehicles/multirotors/__init__.py').read_text()
    (pegasus / 'examples/zav_sim.py').write_text('old file')
    assert subprocess.run(cmd + ['--check'], stdout=subprocess.PIPE).returncode == 1
print('DEPLOYMENT_OK: fresh install, idempotence, original world defaults, drift detection')
