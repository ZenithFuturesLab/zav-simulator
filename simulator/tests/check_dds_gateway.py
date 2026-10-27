"""Exercise actual Fast DDS <-> Cyclone TCP time and external-vision routes."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
ROOT=Path(__file__).resolve().parents[1]

def endpoint(side):
    os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp' if side=='fast' else 'rmw_cyclonedds_cpp'
    if side=='cyclone': os.environ['CYCLONEDDS_URI']=(ROOT/'tools/rviz/cyclonedds_local.xml').as_uri()
    else: os.environ.pop('CYCLONEDDS_URI',None)
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from px4_msgs.msg import SensorCombined,TimesyncStatus,VehicleOdometry
    rclpy.init();node=rclpy.create_node('gateway_contract_'+side)
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
    routes=[(SensorCombined,'/fmu/out/sensor_combined',42001),(TimesyncStatus,'/fmu/out/timesync_status',42002),(VehicleOdometry,'/fmu/in/vehicle_visual_odometry',42003)]
    outgoing=routes[:2] if side=='fast' else routes[2:]
    incoming=routes[2:] if side=='fast' else routes[:2]
    received=set();subs=[];pubs=[]
    for typ,topic,stamp in incoming:
        def callback(msg,t=topic,s=stamp):
            if msg.timestamp==s: received.add(t)
        subs.append(node.create_subscription(typ,topic,callback,qos))
    for typ,topic,stamp in outgoing:
        msg=typ();msg.timestamp=stamp;pubs.append((node.create_publisher(typ,topic,qos),msg))
    deadline=time.monotonic()+18
    while time.monotonic()<deadline:
        for pub,msg in pubs:pub.publish(msg)
        rclpy.spin_once(node,timeout_sec=.04)
    node.destroy_node();rclpy.shutdown()
    assert received=={t for _,t,_ in incoming},(side,received)
    print('GATEWAY_ENDPOINT_OK',side,sorted(received),flush=True)

def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='49','Use isolated domain 49'
    with tempfile.TemporaryDirectory(prefix='zav-gateway-') as temp:
        processes=[];logs=[]
        try:
            for name,argv in [('gateway',[sys.executable,str(ROOT/'tools/runtime/px4_dds_gateway.py')]),('fast',[sys.executable,__file__,'fast']),('cyclone',[sys.executable,__file__,'cyclone'])]:
                log=open(Path(temp)/(name+'.log'),'w+');logs.append((name,log))
                processes.append(subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            results=[p.wait(timeout=28) for p in processes[1:]]
            for name,log in logs:
                log.seek(0);print(name+':\n'+log.read())
            assert results==[0,0],results
        finally:
            for p in processes:
                if p.poll() is None:os.killpg(p.pid,signal.SIGTERM)
            for p in processes:
                try:p.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
            for _,log in logs:log.close()
    print('GATEWAY_OK: SensorCombined, TimesyncStatus, external vision both DDS transports')
if __name__=='__main__':
    if len(sys.argv)>1:endpoint(sys.argv[1])
    else:main()
