#!/usr/bin/env python3
"""Read-only runtime acceptance check. Run after the four launch commands."""
import argparse
import json
import math
from pathlib import Path
import time

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=30)
    parser.add_argument('--output',type=Path,default=Path('.cache/runtime-check.json'))
    args=parser.parse_args()
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from px4_msgs.msg import VehicleOdometry,VehicleLocalPosition
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu,Image,PointCloud2
    from livox_ros_driver2.msg import CustomMsg
    rclpy.init();node=rclpy.create_node('zav_runtime_observer')
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
    counts={};latest={};subscriptions=[]
    def callback(key):
        def receive(msg):
            counts[key]=counts.get(key,0)+1
            if key=='local': latest[key]={'xy_valid':msg.xy_valid,'z_valid':msg.z_valid,'position':[msg.x,msg.y,msg.z]}
            if key=='ev': latest[key]={'position':[float(x) for x in msg.position],'velocity':[float(x) for x in msg.velocity]}
        return receive
    routes=[(VehicleLocalPosition,'/fmu/out/vehicle_local_position','local'),(VehicleOdometry,'/fmu/in/vehicle_visual_odometry','ev'),(Odometry,'/LIVO2/imu_propagate','lio'),(Imu,'/drone0/mid360/imu','imu'),(Image,'/drone0/front_camera/color/image_raw','camera'),(PointCloud2,'/drone0/mid360/pointcloud','cloud'),(CustomMsg,'/drone0/mid360/lidar','livox')]
    for typ,topic,key in routes: subscriptions.append(node.create_subscription(typ,topic,callback(key),qos))
    started=time.monotonic()
    while time.monotonic()-started<args.duration:rclpy.spin_once(node,timeout_sec=.02)
    result={'duration_s':args.duration,'counts':counts,'latest':latest,'nodes':node.get_node_names(),'ev_publishers':node.count_publishers('/fmu/in/vehicle_visual_odometry'),'imu_publishers':node.count_publishers('/drone0/mid360/imu')}
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n')
    node.destroy_node();rclpy.shutdown()
    print(json.dumps(result,indent=2))
    assert all(counts.get(k,0)>5 for k in ('local','ev','lio','imu','camera','cloud','livox')),counts
    assert latest['local']['xy_valid'] and latest['local']['z_valid'],latest
    assert all(math.isfinite(x) for x in latest['ev']['position']+latest['ev']['velocity'])
    assert result['ev_publishers']==1 and result['imu_publishers']==1,result
    print('RUNTIME_OK: sensor streams, FAST-LIVO, EV, EKF position, singleton publishers')
if __name__=='__main__':main()
