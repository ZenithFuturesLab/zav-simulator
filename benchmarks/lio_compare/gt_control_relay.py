#!/usr/bin/env python3
"""Benchmark-only PhysX body GT -> ROS relay for estimator-independent flight."""
import socket,struct,time
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
rclpy.init();node=rclpy.create_node('benchmark_gt_control_relay');pub=node.create_publisher(Odometry,'/benchmark/control_ground_truth',QoSProfile(depth=256,reliability=ReliabilityPolicy.BEST_EFFORT))
def read(c,n):
 out=b''
 while len(out)<n:
  x=c.recv(n-len(out))
  if not x:raise ConnectionError('GT stream closed')
  out+=x
 return out
while rclpy.ok():
 try:
  with socket.create_connection(('127.0.0.1',17447),timeout=3) as c:
   c.settimeout(5);print('GT control connected',flush=True)
   while rclpy.ok():
    n=struct.unpack('!I',read(c,4))[0];assert 0<n<1024*1024
    pub.publish(deserialize_message(read(c,n),Odometry));rclpy.spin_once(node,timeout_sec=0)
 except (OSError,ConnectionError) as e:print(e,flush=True);time.sleep(1)
