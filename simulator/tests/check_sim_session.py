"""A second terminal must join the live simulator's domain, not shell defaults."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
from unittest.mock import patch

source = Path(__file__).resolve().parents[1] / 'tools/runtime/sim_session.py'
spec = importlib.util.spec_from_file_location('sim_session', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
# A real live process identity is verified, not merely the JSON's existence.
module.RUNNER = Path(Path('/proc/self/cmdline').read_bytes().split(b'\0')[0].decode())
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory) / 'session.json'
    with patch.object(module, 'session_path', return_value=path):
        assert module.read_session() is None
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '119', 'ZAV_SUPER_SCENE': 'high_speed',
                                     'ZAV_ROS_TRANSPORT': 'fastdds', 'ROS_LOCALHOST_ONLY': '0'}):
            module.write_session(os.getpid())
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '120'}):
            session = module.wait_for_session('high_speed', timeout=0.)
            assert session['domain'] == 119
        try:
            module.wait_for_session('dense', timeout=0.)
            raise AssertionError('Wrong scene accepted')
        except RuntimeError as exc:
            assert 'high_speed' in str(exc) and 'dense' in str(exc)
        module.clear_session(os.getpid()+1)
        assert path.exists(), 'An unrelated launcher removed the live session'
        data=json.loads(path.read_text());data['pid']=99999999
        path.write_text(json.dumps(data))
        assert module.read_session() is None, 'Stale PID accepted'
        module.clear_session(99999999)
        assert not path.exists()
        try:
            module.wait_for_session('high_speed', timeout=0.)
            raise AssertionError('Missing simulator accepted')
        except RuntimeError:
            pass
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '119', 'ZAV_SUPER_SCENE': 'mixed'}):
            module.write_session(os.getpid())
        assert module.wait_for_session('mixed', timeout=0.)['scene'] == 'mixed'
        module.clear_session(os.getpid())
print('SIM_SESSION_OK: live PID, cross-terminal domain, scene mismatch, stale record and cleanup')
