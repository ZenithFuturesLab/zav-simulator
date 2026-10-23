#!/usr/bin/env python3
"""Configure a checkout for an existing Isaac Sim 5.1.0 installation."""
import argparse
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]

def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--platform', choices=['native', 'wsl'], required=True)
    p.add_argument('--isaac-python', type=Path, help='Native Isaac python.sh or Isaac Python 3.11 venv executable')
    p.add_argument('--isaac-windows-python', type=Path, help='Windows python.bat path as /mnt/c/...')
    p.add_argument('--domain-id', type=int, default=0)
    p.add_argument('--no-shell-integration', action='store_true', help='Do not change home wrappers, bashrc or home config')
    p.add_argument('--validate-only', action='store_true')
    a = p.parse_args()
    if not 0 <= a.domain_id <= 101: p.error('domain-id must be 0..101')
    exe = a.isaac_python if a.platform == 'native' else a.isaac_windows_python
    if not exe or not exe.is_file(): p.error('Provide an existing Isaac Python executable for the selected platform')
    if a.platform == 'native' and not os.access(exe, os.X_OK): p.error('Isaac Python must be executable')
    return a

def patch_px4(px4):
    airframes = px4 / 'ROMFS/px4fmu_common/init.d-posix/airframes'
    shutil.copy2(ROOT/'simulation/px4/10020_isaac_zav', airframes/'10020_isaac_zav')
    cmake = airframes/'CMakeLists.txt'
    text = cmake.read_text()
    if '10020_isaac_zav' not in text:
        marker = 'px4_add_romfs_files('
        if marker not in text: raise RuntimeError('Unsupported airframe CMakeLists')
        text = text.replace(marker, marker+'\n\t10020_isaac_zav', 1)
        cmake.write_text(text)
    cmake = px4/'src/modules/simulation/simulator_mavlink/CMakeLists.txt'
    text = cmake.read_text()
    if 'add_custom_target(isaac_zav' not in text:
        text += '''
# ZAV: launch the external Isaac Sim backend.
add_custom_target(isaac_zav
    COMMAND ${CMAKE_COMMAND} -E env PX4_SYS_AUTOSTART=10020 $<TARGET_FILE:px4>
    WORKING_DIRECTORY ${SITL_WORKING_DIR}
    USES_TERMINAL
    DEPENDS px4
)
'''
        cmake.write_text(text)
    # Fail on ABI drift, instead of silently replacing message definitions.
    for msg in (ROOT/'src/px4_msgs/msg').glob('*.msg'):
        upstream = px4/'msg'/msg.name
        if not upstream.is_file() or msg.read_bytes() != upstream.read_bytes():
            raise RuntimeError(f'PX4 message mismatch: {msg.name}')

def native_isaac_root(exe):
    if exe.name == 'python.sh': return exe.parent
    env = dict(os.environ); env.pop('PYTHONPATH', None)
    code = 'import importlib.util,pathlib; s=importlib.util.find_spec("isaacsim"); print(pathlib.Path(s.origin).parent if s and s.origin else "")'
    result = subprocess.check_output([str(exe), '-c', code], env=env, text=True).strip()
    if not result: raise RuntimeError('This Python does not contain Isaac Sim')
    return Path(result)

def main():
    a = arguments()
    if a.validate_only: return
    settings = {'ZAV_SIM_PLATFORM': a.platform, 'ROS_DOMAIN_ID': str(a.domain_id)}
    if a.platform == 'native':
        exe = a.isaac_python.absolute()
        root = native_isaac_root(exe)
        # Package metadata check does not initialize Kit or require a GPU.
        env = dict(os.environ); env.pop('PYTHONPATH', None)
        probe = subprocess.run([str(exe), '-c', 'import sys; assert sys.version_info[:2]==(3,11); import importlib.metadata as m; print(m.version("isaacsim"))'], env=env, text=True, capture_output=True)
        if probe.returncode == 0 and not probe.stdout.strip().startswith('5.1.0'):
            raise RuntimeError('Expected Isaac Sim 5.1.0: '+probe.stdout)
        # Standalone distributions may lack pip metadata; check their VERSION file.
        if probe.returncode:
            versions = [p.read_text().strip() for p in [root/'VERSION',root/'VERSION.txt'] if p.exists()]
            if not any(v.startswith('5.1.0') for v in versions):
                raise RuntimeError('Cannot verify Isaac Sim 5.1.0 / Python 3.11: '+probe.stderr)
        settings.update(ISAAC_PYTHON=str(exe), ISAAC_SIM_PATH=str(root))
    else:
        settings['ISAAC_WINDOWS_PYTHON_BAT'] = str(a.isaac_windows_python.absolute())
        print('Windows Isaac version must be confirmed in Windows: 5.1.0 with internal Humble.')
    # Upstream uses moving 2.3.x/3.3.x branches. Freeze the tested dependencies.
    agent_cmake = ROOT/'src/Micro-XRCE-DDS-Agent/CMakeLists.txt'
    source = agent_cmake.read_text()
    pins = json.loads((ROOT/'agent-dependencies.lock.json').read_text())
    for name in ('fastcdr', 'fastdds'):
        source = re.sub(r'set\(_'+name+r'_tag [^)]+\)', 'set(_'+name+'_tag '+pins[name]+')', source)
    source = re.sub(r'set\(UAGENT_P2P_CLIENT_TAG [^)]+\)', 'set(UAGENT_P2P_CLIENT_TAG '+pins['microxrcedds_client']+' CACHE STRING "Pinned client")', source)
    agent_cmake.write_text(source)
    superbuild = agent_cmake.parent/'cmake/SuperBuild.cmake'
    source = superbuild.read_text()
    # Always build the pinned DDS/client sources, even on a machine with /usr/local copies.
    for name in ('microxrcedds_client', 'fastcdr', 'fastdds'):
        source = re.sub(r'find_package\('+name+r' [^\n]*QUIET\)', 'set('+name+'_FOUND FALSE) # ZAV: pinned local dependency', source)
    if '-DUCLIENT_BUILD_MICROCDR:BOOL=ON' not in source:
        source = source.replace('-DUCLIENT_ISOLATED_INSTALL:BOOL=ON', '-DUCLIENT_ISOLATED_INSTALL:BOOL=ON\n                -DUCLIENT_SUPERBUILD:BOOL=ON\n                -DUCLIENT_BUILD_MICROCDR:BOOL=ON')
    superbuild.write_text(source)
    patch_px4(ROOT/'src/PX4-Autopilot')
    subprocess.run(['python3',str(ROOT/'simulation/deploy_sim.py'),'--home-dir',str(ROOT/'.cache/wrappers')],check=True)
    (ROOT/'.zav.env').write_text(''.join(f'export {k}={shlex.quote(v)}\n' for k,v in settings.items()))
    if not a.no_shell_integration:
        config = Path(os.environ.get('XDG_CONFIG_HOME', str(Path.home()/'.config')))/'zav/simulation.env'
        config.parent.mkdir(parents=True,exist_ok=True)
        if config.exists() and not config.with_suffix('.env.before-zav-simulator').exists():
            shutil.copy2(config, config.with_suffix('.env.before-zav-simulator'))
        config.write_text('source '+shlex.quote(str(ROOT/'simulation/environment.sh'))+'\n')
        for name in ('run_zav_sim.sh','run_fastlivo.sh','run_mid360_rviz.sh','run_isaac_sim.sh'):
            target=Path.home()/name
            if target.exists() and not target.with_suffix('.sh.before-zav-simulator').exists():
                shutil.copy2(target,target.with_suffix('.sh.before-zav-simulator'))
            target.write_text('#!/usr/bin/env bash\nexec bash '+shlex.quote(str(ROOT/name))+' "$@"\n'); target.chmod(0o755)
        bashrc = Path.home()/'.bashrc'
        text = bashrc.read_text() if bashrc.exists() else ''
        text = re.sub(r'\n# BEGIN ZAV SIMULATOR\n.*?# END ZAV SIMULATOR\n','\n',text,flags=re.S)
        text += '\n# BEGIN ZAV SIMULATOR\nsource '+shlex.quote(str(ROOT/'simulation/ros_setup.sh'))+'\nalias zav_sim='+shlex.quote('bash '+shlex.quote(str(ROOT/'run_zav_sim.sh')))+'\n# END ZAV SIMULATOR\n'
        bashrc.write_text(text)
    print('Configured:', json.dumps(settings,ensure_ascii=False))

if __name__ == '__main__': main()
