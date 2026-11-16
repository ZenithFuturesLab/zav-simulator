"""Sensor-frame and organized-cloud integration checks using real ROS messages."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

spec=importlib.util.spec_from_file_location('lio_adapter',Path(__file__).resolve().parents[1]/'src/lio_adapter.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class Capture:
    def __init__(self): self.messages=[]
    def publish(self,msg): self.messages.append(msg)


class AdapterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()
    def test_rotating_imu_lever_and_world_cloud(self):
        node=module.Adapter();node.scene_ok=True
        node.lever=np.array([.04,.02,.15]);node.odom=Capture();node.cloud=Capture()
        body=Rotation.from_euler('z',.3);imu=body*node.mount
        lever=body.apply(node.lever)
        # IMU linear velocity induced by pure rotation should vanish at body origin.
        omega_world=np.array([0.,0.,1.]);imu_velocity=imu.inv().apply(np.cross(omega_world,lever))
        imu_omega=imu.inv().apply(omega_world)
        msg=Odometry();msg.header.frame_id='camera_init';msg.child_frame_id='aft_mapped';msg.header.stamp.sec=1
        q=imu.as_quat();msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w=map(float,q)
        msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=map(float,lever)
        msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z=map(float,imu_velocity)
        msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z=map(float,imu_omega)
        for i in (0,7,14): msg.twist.covariance[i]=.04
        node.on_odom(msg);self.assertEqual(len(node.odom.messages),1)
        output=node.odom.messages[0]
        p=output.pose.pose.position;v=output.twist.twist.linear
        np.testing.assert_allclose([p.x,p.y,p.z],[0.,-50.,.10],atol=1e-8)
        np.testing.assert_allclose([v.x,v.y,v.z],[0.,0.,0.],atol=1e-8)
        outq=output.pose.pose.orientation
        r=Rotation.from_quat([outq.x,outq.y,outq.z,outq.w])
        np.testing.assert_allclose(r.apply([1.,0.,0.]),[0.,1.,0.],atol=1e-8)
        cloud=PointCloud2();cloud.header.frame_id='camera_init';cloud.height=2;cloud.width=2
        cloud.point_step=16;cloud.row_step=40;cloud.data=bytes(80)
        cloud.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(('x','y','z','intensity'))]
        node.on_cloud(cloud);converted=node.cloud.messages[0]
        self.assertEqual(converted.row_step,40);self.assertEqual(converted.header.frame_id,'world')
        xyz=np.ndarray((2,2,3),dtype='<f4',buffer=bytes(converted.data),strides=(40,16,4))
        np.testing.assert_allclose(xyz,np.broadcast_to(node.shift,(2,2,3)),atol=1e-6)
        node.on_odom(msg);self.assertEqual(len(node.odom.messages),1)
        cloud.width=0;cloud.row_step=0;cloud.data=bytes()
        node.on_cloud(cloud);self.assertEqual(len(node.cloud.messages),1)
        node.destroy_node()

if __name__=='__main__':unittest.main()
