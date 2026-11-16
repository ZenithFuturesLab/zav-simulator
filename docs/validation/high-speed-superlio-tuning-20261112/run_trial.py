"""GUI-only, isolated fresh-simulator trial. Never changes source parameters."""
import argparse,json,os,pty,signal,subprocess,time,fcntl,termios,struct,sys
from pathlib import Path
import yaml
os.environ['ZAV_HEADLESS']='0'
WS=Path(__file__).resolve().parents[3]; ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--boundary',required=True);p.add_argument('--timeout',type=float,default=220);p.add_argument('--reference-config',type=Path);p.add_argument('--robot-radius',type=float);p.add_argument('--inflation-step',type=int);a=p.parse_args()
trial=ROOT/a.name;trial.mkdir(exist_ok=False)
share=WS/'install/zav_super/share/zav_super'
config=share/'config'/('high_speed_super_lio.yaml' if 'high_speed_super_lio.yaml' in (share/'launch/benchmark.launch.py').read_text() else 'high_speed.yaml')
original=config.read_text(); data=yaml.safe_load((a.reference_config or WS/'src/zav_super/config/high_speed.yaml').read_text());data['traj_opt']['boundary'].update(json.loads(a.boundary))
if a.robot_radius is not None:data['super_planner']['robot_r']=a.robot_radius
if a.inflation_step is not None:data['rog_map']['inflation_step']=a.inflation_step
loaded=yaml.safe_dump(data,sort_keys=False);config.write_text(loaded);(trial/'loaded-high_speed.yaml').write_text(loaded)
children=[]; handles=[]
signal.signal(signal.SIGTERM,lambda *_:sys.exit(143))
def launch(cmd,name,terminal=False):
    log=(trial/name).open('w');handles.append(log)
    if terminal:
        master,slave=pty.openpty();fcntl.ioctl(slave,termios.TIOCSWINSZ,struct.pack('HHHH',40,160,0,0));handles.extend([os.fdopen(master,'rb',buffering=0),os.fdopen(slave,'wb',buffering=0)])
        proc=subprocess.Popen(cmd,cwd=WS,stdin=slave,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    else:proc=subprocess.Popen(cmd,cwd=WS,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    children.append(proc);return proc
try:
    sim=launch(['./run_super_sim.sh','high_speed'],'sim-launch.log',True)
    time.sleep(2)
    deadline=time.monotonic()+100
    while time.monotonic()<deadline:
        if sim.poll() is not None:raise RuntimeError('Simulator launcher exited')
        log=Path('/tmp/zav_isaac_sim.log')
        if log.exists() and '[Ready] high_speed' in log.read_text(errors='replace'):break
        time.sleep(1)
    else:raise RuntimeError('Simulator readiness timeout')
    rec=launch(['bash','-c',f'source simulator/ros_setup.sh\npython3 {ROOT}/recorder.py --output {trial}/flight.jsonl --seconds {a.timeout+20}'],'recorder.log')
    nav=launch(['./run_super.sh','--scenario','high_speed','--backend','super_lio','--execute'],'launch.log')
    begin=time.monotonic();reason='timeout';fault_at=None;screenshot=False;midflight=False
    while time.monotonic()-begin<a.timeout:
        time.sleep(1)
        if rec.poll() is not None:raise RuntimeError('Recorder exited')
        if nav.poll() is not None:reason='navigation exited';break
        flight=trial/'flight.jsonl'
        if not flight.exists():continue
        lines=flight.read_text().splitlines()
        try:r=json.loads(lines[-1])
        except (IndexError,json.JSONDecodeError):continue
        state=r.get('controller','')
        if state=='TRACKING' and not screenshot:
            from PIL import ImageGrab
            ImageGrab.grab().save(trial/'gui.png')
            (trial/'windows.txt').write_text(subprocess.check_output(['xwininfo','-root','-tree'],text=True))
            screenshot=True
        if state=='TRACKING' and time.monotonic()-begin>20 and not midflight:
            from PIL import ImageGrab
            ImageGrab.grab().save(trial/'gui-midflight.png');midflight=True
        if state in ('TRACKING','LANDING') and 'truth_imu' in r and 'lio_body' in r:
            import math
            if math.dist(r['truth_imu']['p'],r['lio_body']['p'])>2.0:
                reason='physical/LIO divergence >2m';break
        if state=='LANDED' and r.get('px4',{}).get('armed')==1:reason='landed';break
        if state.startswith(('FAULT','STOPPED')):
            if fault_at is None:fault_at=time.monotonic()
            if time.monotonic()-fault_at>12:reason=state;break
        if int(time.monotonic()-begin)%10==0:print(a.name,round(time.monotonic()-begin),state,r.get('lio_body',{}).get('p'),flush=True)
    (trial/'result.json').write_text(json.dumps({'termination':reason,'wall_s':time.monotonic()-begin},indent=2))
    print(a.name,reason,flush=True)
finally:
    for child in reversed(children):
        if child.poll() is None:
            try:os.killpg(child.pid,signal.SIGINT if child is not children[0] else signal.SIGTERM)
            except ProcessLookupError:pass
            try:child.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL)
    log=Path('/tmp/zav_isaac_sim.log')
    if log.exists():(trial/'isaac.log').write_bytes(log.read_bytes())
    config.write_text(original)
