#!/usr/bin/env python3
"""Run the selected LIO, real sensor relay and SUPER as one supervised session."""
import argparse
import os
from pathlib import Path
import sys
import subprocess
from datetime import datetime
from zoneinfo import ZoneInfo
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'runtime'))
from fastlivo_session import run_session, check_existing_nodes
from sim_session import wait_for_session


def preflight(workspace):
    check_existing_nodes(workspace)
    conflicts=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit(): continue
        try:
            args=(proc/'cmdline').read_bytes().decode().split('\0')
            if any(Path(x).name in ('flight_manager','ommpc_px4') for x in args if x): conflicts.append(proc.name)
        except (OSError,UnicodeError): pass
    if conflicts: raise RuntimeError(f'Another offboard controller is running: {conflicts}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',choices=('high_speed','dense','mixed'),default='high_speed')
    parser.add_argument('--backend',choices=('super_lio','fastlivo','fastlio'),default='super_lio')
    parser.add_argument('--profile',choices=('legacy','common'),default='legacy',
                        help='common: one shared Super-LIO planner/controller YAML for all benchmark scenes')
    parser.add_argument('--config',type=Path,
                        help='Experimental shared YAML override; requires --profile common')
    parser.add_argument('--execute',action='store_true',help='Arm, take off and fly the original benchmark round trip')
    parser.add_argument('--no-rviz',action='store_true')
    parser.add_argument('--record-dir',type=Path,help='Flight data directory (default: log/super_flights/<timestamp>)')
    args=parser.parse_args()
    if args.scenario=='mixed' and args.profile!='common':
        parser.error('mixed requires --profile common (one YAML across all zones)')
    if args.profile=='common' and args.backend!='super_lio':
        parser.error('--profile common currently requires --backend super_lio')
    if args.config:
        if args.profile!='common':parser.error('--config requires --profile common')
        args.config=args.config.expanduser().resolve()
        if not args.config.is_file():parser.error(f'Configuration file not found: {args.config}')
    ws=Path(os.environ['ZAV_WS_DIR'])
    print('[SUPER] Waiting for simulator session...', flush=True)
    session=wait_for_session(args.scenario)
    os.environ['ROS_DOMAIN_ID']=str(session['domain'])
    os.environ['ROS_LOCALHOST_ONLY']=session['localhost_only']
    os.environ['ZAV_ROS_TRANSPORT']=session['transport']
    print(f"[SUPER] scene={session['scene']} ROS_DOMAIN_ID={session['domain']} "
          f"(simulator launcher PID {session['pid']})", flush=True)
    if os.environ.get('ZAV_ROS_TRANSPORT')=='cyclone_tcp':
        raise RuntimeError('SUPER currently requires native DDS; the existing WSL isolated domains need additional topic routes')
    relay=[sys.executable,str(ws/'simulator/tools/runtime/sensor_relay.py'),'client',
           '--host',os.environ.get('ZAV_SENSOR_HOST','127.0.0.1'),'--camera-port','17443' if args.backend=='fastlivo' else '0',
           '--imu-port','17444','--livox-port','17445']
    command=['ros2','launch','zav_super','benchmark.launch.py',f'scenario:={args.scenario}',
             f'backend:={args.backend}',f'profile:={args.profile}',f'execute:={str(args.execute).lower()}',f'rviz:={str(not args.no_rviz).lower()}']
    if args.config:command.append(f'common_config:={args.config}')
    output=(args.record_dir or ws/'log/super_flights'/datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d-%H%M%S-%f')).resolve()
    if output.exists():parser.error(f'--record-dir must be a new directory: {output}')
    output.mkdir(parents=True,exist_ok=False)
    command.append(f'telemetry_dir:={output}')
    print(f'[SUPER] Flight data: {output} (report.html / motion.png / summary.json / kinematics.csv)',flush=True)
    result=run_session(relay,command,f'/tmp/zav-fastlivo-{os.getuid()}.lock',log_path='/tmp/zav-super-relay.log',
                       preflight=lambda:preflight(ws))
    # Supervision may terminate the recorder before plotting during Ctrl+C.
    raw=output/'samples.jsonl';report=output/'report.html'
    if raw.exists() and (not report.exists() or report.stat().st_mtime<raw.stat().st_mtime):
        subprocess.run([sys.executable,str(ws/'simulator/tools/super/flight_metrics.py'),str(output)],check=False)
    return result

if __name__=='__main__':
    try: raise SystemExit(main())
    except RuntimeError as e: print(e,file=sys.stderr);raise SystemExit(1)
