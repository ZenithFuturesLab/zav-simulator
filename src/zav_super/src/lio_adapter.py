#!/usr/bin/env python3
"""Selected LIO IMU pose -> SUPER world/body pose; measured map -> same world.

The constant alignment anchors the first measured body pose at the known spawn.
No ground truth is used as feedback; twist is rotated from the IMU child frame
and corrected for omega cross lever-arm before conversion to world velocity.
"""
import copy
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Imu, PointField
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation


class Adapter(Node):
    def __init__(self):
        super().__init__('zav_super_lio_adapter')
        self.declare_parameter('scenario', 'high_speed')
        self.scene_ok=False
        self.create_subscription(String, '/zav/sim/scenario', self.on_scene, 1)
        self.declare_parameter('odom_topic', '/LIVO2/imu_propagate')
        self.declare_parameter('cloud_topic', '/cloud_registered')
        self.declare_parameter('spawn', [0., -50., .10])
        self.declare_parameter('spawn_yaw', np.pi/2)
        self.declare_parameter('imu_position', [0., 0., 0.])
        self.declare_parameter('imu_orientation', [0., 22., 0.])
        self.mount = Rotation.from_euler('ZYX', self.get_parameter('imu_orientation').value, degrees=True)
        self.lever = np.array(self.get_parameter('imu_position').value)
        self.alignment = None
        self.shift = None
        self.last_stamp = 0.
        self.odom = self.create_publisher(Odometry, '/zav/super/odom', 10)
        self.cloud = self.create_publisher(PointCloud2, '/zav/super/cloud_world', 2)
        self.clock = self.create_publisher(Clock, '/clock', 10)
        self.robot = self.create_publisher(Marker, '/robot', 2)
        self.tf = TransformBroadcaster(self)
        q = qos_profile_sensor_data
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, q)
        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value, self.on_cloud, q)
        self.create_subscription(Imu, '/drone0/mid360/imu', self.on_imu, q)

    def on_scene(self,msg):
        self.scene_ok=msg.data==self.get_parameter('scenario').value
        if not self.scene_ok:
            self.get_logger().error('Isaac scene and SUPER scenario differ; restart Isaac with run_super_sim.sh')

    def on_imu(self, msg):
        clock = Clock(); clock.clock = msg.header.stamp
        self.clock.publish(clock)

    def on_odom(self, msg):
        if not self.scene_ok: return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if stamp <= self.last_stamp:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        v,w = msg.twist.twist.linear, msg.twist.twist.angular
        if not np.isfinite(values).all() or np.linalg.norm(values[3:]) < .9:
            return
        # Both current LIO backends explicitly publish the mounted-IMU contract.
        if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'aft_mapped':
            self.get_logger().error('Unexpected LIO frames; refusing body conversion')
            return
        if not all(msg.twist.covariance[i] > 0 for i in (0,7,14)):
            return  # Unknown velocity is unsuitable for MPC.
        r_wi = Rotation.from_quat(values[3:])
        r_wb = r_wi * self.mount.inv()
        lever_world = r_wb.apply(self.lever)
        body_p = values[:3] - lever_world
        if self.alignment is None:
            yaw = np.arctan2(r_wb.as_matrix()[1,0], r_wb.as_matrix()[0,0])
            self.alignment = Rotation.from_euler('z', self.get_parameter('spawn_yaw').value-yaw)
            self.shift = np.array(self.get_parameter('spawn').value)-self.alignment.apply(body_p)
        vel = r_wi.apply([v.x,v.y,v.z])-np.cross(r_wi.apply([w.x,w.y,w.z]),lever_world)
        out=copy.deepcopy(msg)
        out.header.frame_id='world';out.child_frame_id='zav_body'
        pos=self.alignment.apply(body_p)+self.shift
        quat=(self.alignment*r_wb).as_quat()
        out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,pos)
        out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,quat)
        # Adapter output explicitly uses world linear velocity, FLU body rates.
        vel=self.alignment.apply(vel);omega=self.mount.apply([w.x,w.y,w.z])
        out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=map(float,vel)
        out.twist.twist.angular.x,out.twist.twist.angular.y,out.twist.twist.angular.z=map(float,omega)
        rot=self.alignment.as_matrix()
        for field in (out.pose, out.twist):
            cov=np.array(field.covariance).reshape(6,6)
            transform=np.zeros((6,6));transform[:3,:3]=rot;transform[3:,3:]=rot
            field.covariance=(transform@cov@transform.T).flatten().tolist()
        self.odom.publish(out)
        self.last_stamp=stamp
        tf=TransformStamped();tf.header=out.header;tf.child_frame_id='zav_body'
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,pos)
        tf.transform.rotation=out.pose.pose.orientation;self.tf.sendTransform(tf)
        # Four actual guard locations + body envelope, not a point robot.
        marker=Marker();marker.header=out.header;marker.ns='zav';marker.id=0
        marker.type=Marker.MESH_RESOURCE;marker.mesh_resource="package://zav_super/meshes/zav.stl";marker.mesh_use_embedded_materials=False;marker.action=Marker.ADD;marker.pose=out.pose.pose
        marker.scale.x=marker.scale.y=marker.scale.z=1.
        marker.color.r=.1;marker.color.g=.6;marker.color.b=1.;marker.color.a=1.
        self.robot.publish(marker)

    def on_cloud(self, msg):
        if self.alignment is None or msg.header.frame_id != 'camera_init':
            return
        if msg.width == 0 or msg.height == 0:
            return
        fields={f.name:f for f in msg.fields}
        if any(k not in fields or fields[k].datatype != PointField.FLOAT32 for k in ('x','y','z')):
            return
        needed=(msg.height-1)*msg.row_step+(msg.width-1)*msg.point_step+max(fields[k].offset for k in ('x','y','z'))+4
        if len(msg.data)<needed or msg.row_step<msg.width*msg.point_step:
            self.get_logger().error('Malformed PointCloud2 layout; dropping sample')
            return
        result=copy.deepcopy(msg);data=bytearray(msg.data)
        endian='>' if msg.is_bigendian else '<'
        # Respect organized cloud row padding and retain every non-XYZ field.
        views=[np.ndarray((msg.height,msg.width), dtype=endian+'f4', buffer=data,
                          offset=fields[k].offset, strides=(msg.row_step,msg.point_step)) for k in ('x','y','z')]
        points=np.stack(views,axis=-1)
        transformed=self.alignment.apply(points.reshape(-1,3)).reshape(points.shape)+self.shift
        for i,view in enumerate(views): view[:]=transformed[...,i]
        result.data=bytes(data);result.header.frame_id='world'
        self.cloud.publish(result)


def main():
    rclpy.init();node=Adapter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
