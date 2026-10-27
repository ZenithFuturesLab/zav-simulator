"""Isolated integration: sensor TCP -> actual ROS types + PX4 feedback QoS.

Run after sourcing ROS/workspace, with ROS_DOMAIN_ID=49 (never the live domain).
No Isaac GUI, real PX4, arming command or hardware SDK is started.
"""
import contextlib
import importlib.util
import io
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from array import array

assert os.environ.get('ROS_DOMAIN_ID') == '49', 'Run in isolated ROS_DOMAIN_ID=49'
assert os.environ.get('RMW_IMPLEMENTATION') in ('rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp')
cyclone = os.environ['RMW_IMPLEMENTATION'] == 'rmw_cyclonedds_cpp'
if cyclone:
    os.environ['CYCLONEDDS_URI'] = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast><Transport>tcp</Transport></General><Discovery><Peers><Peer Address="127.0.0.1:17542"/></Peers></Discovery><TCP><Port>17542</Port></TCP></Domain></CycloneDDS>'
os.environ['ZAV_SIM_PLATFORM'] = 'native'
import rclpy
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField, Image, Imu
from livox_ros_driver2.msg import CustomMsg
from px4_msgs.msg import VehicleLocalPosition

root = Path(__file__).resolve().parents[1]
stop = threading.Event()
threads, listeners = [], []
cloud = PointCloud2(height=1, width=1, point_step=12, row_step=12)
cloud.header.frame_id = 'mid360'
cloud.header.stamp.nanosec = 100_000_000
cloud.fields = [PointField(name=a, offset=i*4, datatype=PointField.FLOAT32, count=1) for i,a in enumerate('xyz')]
cloud.data = array('B', struct.pack('<fff', 1.,2.,3.))
image = Image(height=2, width=2, encoding='rgb8', step=6, data=array('B', [100]*12))
image.header.frame_id = 'front_camera_optical'
imu = Imu(); imu.header.frame_id = 'mid360_imu'; imu.linear_acceleration.z = 9.81
livox = struct.pack('<QI', 100_000_000, 1) + struct.pack('<fffIBBBB', 1.,2.,3.,0,128,0,0x10,0)
ports = []
for payload in (serialize_message(cloud), serialize_message(image), serialize_message(imu), livox):
    listener = socket.socket(); listener.bind(('127.0.0.1',0)); listener.listen(); listener.settimeout(.2)
    listeners.append(listener); ports.append(listener.getsockname()[1])
    def send(listener=listener, payload=payload):
        while not stop.is_set():
            try: conn,_ = listener.accept()
            except socket.timeout: continue
            with conn:
                conn.settimeout(.5)
                try:
                    while not stop.wait(.05): conn.sendall(struct.pack('!I',len(payload))+payload)
                except OSError: pass
    thread = threading.Thread(target=send, daemon=True); thread.start(); threads.append(thread)

rclpy.init(); node = rclpy.create_node('verify_native_sensor_transport')
counts, received = {}, {}
def cb(key):
    def receive(msg): counts[key]=counts.get(key,0)+1; received[key]=msg
    return receive
q = QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT)
subscriptions = [node.create_subscription(t,p,cb(k),q) for t,p,k in (
    (PointCloud2,'/drone0/mid360/pointcloud','cloud'), (Image,'/drone0/front_camera/color/image_raw','image'),
    (Imu,'/drone0/mid360/imu','imu'), (CustomMsg,'/drone0/mid360/lidar','livox'))]
spec = importlib.util.spec_from_file_location('flight', root.parent / 'src/zav_control/zav_control/zav_flight_manager.py')
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
with contextlib.redirect_stdout(io.StringIO()): manager = module.ZAVFlightManager()
# VOLATILE publisher reproduces the direct native PX4 endpoint (no WSL gateway).
position_pub = node.create_publisher(VehicleLocalPosition,'/fmu/out/vehicle_local_position',q)
position = VehicleLocalPosition(xy_valid=True,z_valid=True,z=-2.)
if '--qos-only' in sys.argv:
    try:
        end = time.monotonic() + 3
        while time.monotonic() < end and not manager.position_ready:
            position_pub.publish(position)
            rclpy.spin_once(manager, timeout_sec=.05)
        assert manager.position_ready
        print('PX4_NATIVE_QOS_OK: real VOLATILE publisher accepted by flight_manager')
    finally:
        stop.set()
        for thread in threads:thread.join(timeout=1)
        for listener in listeners:listener.close()
        manager.multi_key.stop();manager.destroy_node();node.destroy_node();rclpy.shutdown()
    raise SystemExit(0)
child_env = os.environ.copy()
if cyclone:
    child_env['CYCLONEDDS_URI'] = os.environ['CYCLONEDDS_URI'].replace('<Port>17542</Port>', '<Port>17546</Port>')
with tempfile.TemporaryFile(mode='w+') as log:
    receiver = subprocess.Popen([sys.executable,str(root/'tools/runtime/sensor_relay.py'),'client',
                                '--port',str(ports[0]),'--camera-port',str(ports[1]),
                                '--imu-port',str(ports[2]),'--livox-port',str(ports[3])],stdout=log,stderr=log,env=child_env)
    try:
        deadline = time.monotonic()+15
        while time.monotonic()<deadline:
            position_pub.publish(position)
            rclpy.spin_once(node,timeout_sec=.01); rclpy.spin_once(manager,timeout_sec=.01)
            if all(counts.get(k,0)>=5 for k in ('cloud','image','imu','livox')) and manager.position_ready: break
        assert all(counts.get(k,0)>=5 for k in ('cloud','image','imu','livox')), counts
        assert manager.position_ready and abs(manager.current_alt-2.)<1e-6
        assert received['cloud'].width==1 and len(received['cloud'].data)==12
        assert received['image'].encoding=='rgb8' and len(received['image'].data)==12
        assert received['imu'].linear_acceleration.z==9.81
        assert received['livox'].point_num==1 and received['livox'].points[0].tag==0x10
        print('ROS_SENSOR_TRANSPORT_OK', os.environ['RMW_IMPLEMENTATION'], counts, 'PX4 volatile feedback accepted without gateway')
    except Exception:
        log.seek(0); print(log.read()); raise
    finally:
        receiver.terminate()
        try: receiver.wait(timeout=8)
        except subprocess.TimeoutExpired: receiver.kill();receiver.wait()
        stop.set()
        for thread in threads:thread.join(timeout=1)
        for listener in listeners:listener.close()
        manager.multi_key.stop(); manager.destroy_node();node.destroy_node();rclpy.shutdown()
