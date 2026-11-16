#!/usr/bin/env python3
"""Record feedback, physical ground truth, reference and rate-only PX4 mode.

Ground truth is subscribed ONLY in this diagnostic, never in control/planning.
"""
import argparse
import json
from pathlib import Path
import time
import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as qos
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from px4_msgs.msg import VehicleRatesSetpoint, VehicleControlMode, VehicleStatus
from mars_quadrotor_msgs.msg import PolynomialTrajectory


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=180.)
    args=parser.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'simulator/tools/runtime'))
    from sim_session import read_session
    session=read_session()
    if session:
        os.environ['ROS_DOMAIN_ID']=str(session['domain'])
        os.environ['ROS_LOCALHOST_ONLY']=session['localhost_only']
    rclpy.init();node=Node('zav_super_validation_recorder');latest={}
    events=args.output.with_name('odom-events.jsonl').open('w')
    def odom(key,msg):
        p,q,v=msg.pose.pose.position,msg.pose.pose.orientation,msg.twist.twist.linear
        latest[key]={'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                     'p':[p.x,p.y,p.z],'q':[q.w,q.x,q.y,q.z],'v':[v.x,v.y,v.z]}
        events.write(json.dumps({'rx':time.monotonic(), 'key':key, **latest[key]})+'\n');events.flush()
    for topic,key in [('/zav/super/odom','lio_body'),('/drone0/ground_truth/odometry','truth_imu')]:
        node.create_subscription(Odometry,topic,lambda m,k=key:odom(k,m),qos)
    def rates(m):latest['rates']=[m.roll,m.pitch,m.yaw,*list(map(float,m.thrust_body))]
    def mode(m):latest['mode']={'rates':m.flag_control_rates_enabled,'attitude':m.flag_control_attitude_enabled,'position':m.flag_control_position_enabled,'velocity':m.flag_control_velocity_enabled,'acceleration':m.flag_control_acceleration_enabled,'altitude':m.flag_control_altitude_enabled}
    def status(m):latest['px4']={'nav':m.nav_state,'armed':m.arming_state}
    def poly(m):
        if m.type & m.POSITION_TRAJ:
            latest['trajectory']={'start':m.start_wt_pos,'durations':list(m.time_pos),'x':list(m.coef_pos_x),'y':list(m.coef_pos_y),'z':list(m.coef_pos_z),'yaw':list(m.coef_yaw),'yaw_start':m.start_wt_yaw,'yaw_durations':list(m.time_yaw)}
    node.create_subscription(VehicleRatesSetpoint,'/fmu/in/vehicle_rates_setpoint',rates,qos)
    node.create_subscription(VehicleControlMode,'/fmu/out/vehicle_control_mode',mode,qos)
    node.create_subscription(VehicleStatus,'/fmu/out/vehicle_status',status,qos)
    node.create_subscription(String,'/zav/super/controller_status',lambda m:latest.update(controller=m.data),10)
    node.create_subscription(PolynomialTrajectory,'/planning_cmd/poly_traj',poly,qos)
    begin=time.monotonic();last=0.
    with args.output.open('w') as file:
        try:
            while rclpy.ok() and time.monotonic()-begin<args.seconds:
                rclpy.spin_once(node,timeout_sec=.05)
                elapsed=time.monotonic()-begin
                if elapsed-last>=.1:
                    file.write(json.dumps({'wall_elapsed':elapsed,**latest})+'\n');file.flush();last=elapsed
        except KeyboardInterrupt:pass
    node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
