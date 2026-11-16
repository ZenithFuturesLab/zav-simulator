"""Isolated ROS adapter checks, not a flight or simulator performance test.

Run after stopping navigation: source simulator/ros_setup.sh, then run this file.
Each case uses its own localhost-only DDS domain and synthetic feedback. The
controller's existing global offboard lock prevents concurrent navigation.
"""
import os
os.environ['ROS_LOCALHOST_ONLY']='1'
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as qos
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from px4_msgs.msg import SensorCombined, VehicleStatus, VehicleRatesSetpoint, VehicleLandDetected, VehicleCommand
from mars_quadrotor_msgs.msg import PolynomialTrajectory
from std_msgs.msg import String
from std_srvs.srv import Trigger


class TerminalHoldROS(unittest.TestCase):
    def exercise(self, domain, enabled, duration, terminal_velocity=False, stale=None, px4_landing=False):
        context=Context();rclpy.init(context=context,domain_id=domain)
        node=Node('terminal_hold_test',context=context)
        executor=SingleThreadedExecutor(context=context);executor.add_node(node)
        odom=node.create_publisher(Odometry,'/zav/super/odom',qos)
        sensor=node.create_publisher(SensorCombined,'/fmu/out/sensor_combined',qos)
        vehicle=node.create_publisher(VehicleStatus,'/fmu/out/vehicle_status',qos)
        contact=node.create_publisher(VehicleLandDetected,'/fmu/out/vehicle_land_detected',qos)
        trajectory=node.create_publisher(PolynomialTrajectory,'/planning_cmd/poly_traj',qos)
        states=[];rates=[];commands=[]
        node.create_subscription(String,'/zav/super/controller_status',lambda m:states.append(m.data),10)
        node.create_subscription(VehicleRatesSetpoint,'/fmu/in/vehicle_rates_setpoint',lambda m:rates.append(m.timestamp),qos)
        node.create_subscription(VehicleCommand,'/fmu/in/vehicle_command',lambda m:commands.append(m.command),qos)
        start=node.create_client(Trigger,'/zav/super/start')
        land=node.create_client(Trigger,'/zav/super/land')
        begun=time.monotonic();feedback={'odom':True,'px4':True,'x':0.,'z':1.5,
                                        'armed':VehicleStatus.ARMING_STATE_ARMED,
                                        'nav':VehicleStatus.NAVIGATION_STATE_OFFBOARD,'landed':False}
        def publish():
            elapsed=time.monotonic()-begun;stamp=elapsed+10.
            if feedback['odom']:
                msg=Odometry();msg.header.frame_id='world';msg.child_frame_id='zav_body'
                msg.header.stamp.sec=int(stamp);msg.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
                msg.pose.pose.position.x=feedback['x'];msg.pose.pose.position.z=feedback['z'];msg.pose.pose.orientation.w=1.
                odom.publish(msg)
            if feedback['px4']:
                msg=SensorCombined();msg.timestamp=int(time.time()*1e6);sensor.publish(msg)
            msg=VehicleStatus();msg.arming_state=feedback['armed'];msg.nav_state=feedback['nav']
            vehicle.publish(msg)
            msg=VehicleLandDetected();msg.landed=feedback['landed'];contact.publish(msg)
        node.create_timer(.01,publish)
        def spin(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
        binary=Path(get_package_prefix('zav_super'))/'lib/zav_super/ommpc_px4'
        env=dict(os.environ,ROS_DOMAIN_ID=str(domain))
        with tempfile.TemporaryFile(mode='w+') as log:
            process=subprocess.Popen([str(binary),'--ros-args','-p',f'allow_terminal_hold:={str(enabled).lower()}',
                                      '-p','step_T:=0.05','-p','control_period:=0.05',
                                      '-p',f'use_px4_landing:={str(px4_landing).lower()}'],env=env,stdout=log,stderr=subprocess.STDOUT)
            try:
                end=time.monotonic()+8
                while not start.service_is_ready() and time.monotonic()<end:
                    self.assertIsNone(process.poll(),'Controller exited; stop existing navigation before this test')
                    spin(.05)
                self.assertTrue(start.service_is_ready())
                spin(.2);future=start.call_async(Trigger.Request())
                end=time.monotonic()+3
                while not future.done() and time.monotonic()<end:spin(.02)
                self.assertTrue(future.done());self.assertTrue(future.result().success)
                spin(.25)
                msg=PolynomialTrajectory();msg.header.frame_id='world';msg.type=msg.POSITION_TRAJ|msg.HEART_BEAT
                msg.order_pos=7;msg.piece_num_pos=1;msg.time_pos=[duration]
                msg.coef_pos_x=[0.]*8;msg.coef_pos_y=[0.]*8;msg.coef_pos_z=[0.]*7+[1.5]
                if terminal_velocity:msg.coef_pos_x[6]=1.;feedback['x']=duration
                msg.start_wt_pos=time.monotonic()-begun+10.;trajectory.publish(msg)
                spin(2.2)
                if enabled and duration<1 and not terminal_velocity:
                    self.assertFalse(any(s.startswith('FAULT')for s in states),states)
                    self.assertIn('TRACKING',states);self.assertGreater(len(rates),20)
                    log.flush();log.seek(0);self.assertIn('terminal stop verified',log.read())
                    if stale:
                        feedback[stale]=False;spin(1.3)
                        self.assertIn('FAULT: Stale LIO/PX4 feedback',states)
                    if px4_landing:
                        future=land.call_async(Trigger.Request());spin(.15)
                        self.assertTrue(future.done());self.assertTrue(future.result().success)
                        self.assertIn(VehicleCommand.VEHICLE_CMD_NAV_LAND,commands)
                        feedback['nav']=VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                        feedback['z']=2.0  # LIO ground-height discrepancy must not decide touchdown.
                        feedback['landed']=True;spin(.15)
                        self.assertNotIn('LANDED',states)  # still armed
                        count=len(rates);spin(.1);self.assertEqual(len(rates),count)
                        feedback['armed']=VehicleStatus.ARMING_STATE_STANDBY
                        feedback['landed']=False;spin(.15);self.assertNotIn('LANDED',states)
                        feedback['landed']=True;spin(.15);self.assertIn('LANDED',states)
                        self.assertNotIn(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,commands)
                        self.assertFalse(any(s.startswith('FAULT')for s in states),states)
                else:self.assertIn('FAULT: SUPER trajectory heartbeat expired',states)
            finally:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait()
                executor.remove_node(node);executor.shutdown()
                node.destroy_node();context.shutdown()

    def test_legacy_stop_still_requires_heartbeat(self):self.exercise(225,False,.3)
    def test_terminal_hold_keeps_odom_freshness_guard(self):self.exercise(226,True,.3,stale='odom')
    def test_terminal_hold_keeps_px4_freshness_guard(self):self.exercise(227,True,.3,stale='px4')
    def test_unfinished_reference_still_requires_heartbeat(self):self.exercise(228,True,5.)
    def test_nonzero_terminal_velocity_still_requires_heartbeat(self):self.exercise(229,True,.3,terminal_velocity=True)
    def test_px4_landing_requires_contact_and_disarm(self):self.exercise(230,True,.3,px4_landing=True)

if __name__=='__main__':unittest.main(verbosity=2)
