"""Exercise real relay/trigger processes on isolated Cyclone TCP ports/domain.

Only synthetic Odometry/status are used; no /fmu control topic is published.
"""
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def main():
    os.environ.update(ROS_DOMAIN_ID='49', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      ZAV_ROS_TRANSPORT='cyclone_tcp')
    monitor_port, relay_port = free_port(), free_port()
    assert monitor_port != relay_port
    with tempfile.TemporaryDirectory(prefix='lio-tcp-test-') as temp:
        temp = Path(temp)
        runtime, rviz = temp / 'tools/runtime', temp / 'tools/rviz'
        runtime.mkdir(parents=True)
        rviz.mkdir()
        script = runtime / 'lio_safety_experiment.py'
        shutil.copy2(ROOT / 'tools/runtime/lio_safety_experiment.py', script)
        for name in ('relay', 'trigger'):
            source = (ROOT / f'tools/rviz/cyclonedds_experiment_{name}.xml').read_text()
            source = source.replace(':17446', f':{monitor_port}').replace(':17451', f':{monitor_port}')
            source = source.replace(':17452', f':{relay_port}').replace('<Port>17452</Port>', f'<Port>{relay_port}</Port>')
            (rviz / f'cyclonedds_experiment_{name}.xml').write_text(source)
        monitor = (rviz / 'cyclonedds_experiment_trigger.xml').read_text().replace(
            '<Port>0</Port>', f'<Port>{monitor_port}</Port>')
        os.environ['CYCLONEDDS_URI'] = monitor
        import rclpy
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        rclpy.init()
        node = rclpy.create_node('synthetic_lio_experiment_monitor')
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        odom_pub = node.create_publisher(Odometry, '/LIVO2/imu_propagate', qos)
        status_pub = node.create_publisher(String, '/zav/lio_safety/status', retained)
        health = dict(state='READY', ready=True, armed=False, latched=False,
                      landing_mode_confirmed=False, landing_complete=False, land_ack=None)
        observed = []

        def shadow(msg):
            if msg.pose.pose.position.x > 2 or abs(msg.pose.pose.orientation.z) > .5:
                observed.append(msg)
                health.update(state='FAILED', ready=False, armed=False, latched=True,
                              landing_mode_confirmed=True, landing_complete=True, land_ack=0)

        subscription = node.create_subscription(Odometry, '/zav/lio_safety/test/odom', shadow, qos)
        original = Odometry()
        original.pose.pose.orientation.w = 1.
        processes = []
        logs = []

        def pump(seconds, process=None):
            until = time.monotonic() + seconds
            while time.monotonic() < until and (process is None or process.poll() is None):
                odom_pub.publish(original)
                status_pub.publish(String(data=json.dumps(health)))
                rclpy.spin_once(node, timeout_sec=.03)

        def spawn(arguments):
            log = (temp / f'process-{len(logs)}.log').open('w+')
            logs.append(log)
            process = subprocess.Popen([sys.executable, str(script), *arguments], stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(process)
            return process, log

        try:
            relay, relay_log = spawn(['relay'])
            pump(1.5)
            rejected, log = spawn(['trigger', 'position-step', '--sitl', '--discovery-timeout', '5'])
            pump(9, rejected)
            log.seek(0)
            output = log.read()
            assert rejected.poll() == 1 and 'must be armed' in output, output
            assert not observed, 'Disarmed refusal must not inject a fault'
            print('PASS: relay/status discovered; disarmed trigger refused', flush=True)
            for fault in ('position-step', 'attitude-step'):
                # Drain the previous short injection before resetting the fake monitor.
                pump(.8)
                health.update(state='READY', ready=True, armed=True, latched=False,
                              landing_mode_confirmed=False, landing_complete=False, land_ack=None)
                pump(1.2)
                before = len(observed)
                trigger, log = spawn(['trigger', fault, '--sitl', '--duration', '.3',
                                      '--observe', '5', '--discovery-timeout', '5'])
                pump(12, trigger)
                log.seek(0)
                output = log.read()
                assert trigger.poll() == 0 and 'PASS: failsafe' in output, output
                assert len(observed) > before, f'{fault} did not reach relay'
                print(f'PASS: {fault} delivered through real relay and result observed', flush=True)
            assert relay.poll() is None
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            for log in logs:
                log.close()
            node.destroy_node()
            rclpy.shutdown()
    print('LIO_EXPERIMENT_TCP_OK: isolated domain 49 and ephemeral fixture ports', flush=True)


if __name__ == '__main__':
    main()
