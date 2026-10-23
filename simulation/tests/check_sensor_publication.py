"""Regression: actual backend methods publish once, via TCP OR direct DDS (no Isaac/GPU required)."""
import ast
from array import array
import contextlib
import io
from pathlib import Path
from types import SimpleNamespace, MethodType
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField

source = Path(__file__).resolve().parents[1] / 'pegasus/overlay/logic/backends/ros2_backend.py'
cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef) and n.name=='ROS2Backend')
names = ('_publish_mounted_imu','_publish_python_camera','_publish_python_lidar')
methods = [m for m in cls.body if isinstance(m,ast.FunctionDef) and m.name in names]
class Annotator:
    def attach(self, render): pass
    def get_data(self):return {'data':np.array([[1.,2.,3.]],dtype=np.float32)}
rep = SimpleNamespace(create=SimpleNamespace(render_product=lambda *a,**kw:'render'),
                      AnnotatorRegistry=SimpleNamespace(get_annotator=lambda name:Annotator()))
namespace = dict(np=np, array=array, Imu=Imu, Image=Image, PointCloud2=PointCloud2, PointField=PointField,
                 serialize_message=serialize_message, QoSProfile=QoSProfile,
                 ReliabilityPolicy=ReliabilityPolicy, qos_profile_sensor_data=qos_profile_sensor_data, rep=rep)
exec(compile(ast.Module(body=methods,type_ignores=[]),str(source),'exec'),namespace)
class Stream:
    def __init__(self):self.frames=[]
    def submit(self,payload):self.frames.append(payload)
class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)
class Node:
    def __init__(self, allow):self.allow=allow;self.publishers=[]
    def create_publisher(self,*a,**kw):
        assert self.allow,'TCP mode must not create a second DDS publisher'
        p=Publisher();self.publishers.append(p);return p
for tcp in (True,False):
    streams = {key:Stream() for key in ['imu:/drone0/mid360/imu','camera:front_camera','mid360']}
    node=Node(allow=not tcp)
    backend=SimpleNamespace(_namespace='drone',_id=0,node=node,_lidar_streams=streams,
                            _mounted_imu_publishers={},_python_cameras={},_python_lidars={},
                            _lidar_first_cloud_logged=set(),
                            _vehicle=SimpleNamespace(_world=SimpleNamespace(current_time=.123)))
    for name in names:setattr(backend,name,MethodType(namespace[name],backend))
    port=17444 if tcp else None
    with contextlib.redirect_stdout(io.StringIO()):
        backend._publish_mounted_imu({'config':{'tcp_port':port,'frame_id':'mid360_imu'},
                                     'angular_velocity':[0,0,0],'linear_acceleration':[0,0,9.81],
                                     'gyro_variance':0.,'accel_variance':0.})
        backend._publish_python_camera({'camera_name':'front_camera','tcp_port':port,
                                        'camera':SimpleNamespace(get_rgba=lambda:np.zeros((2,2,4),dtype=np.uint8))})
        backend._publish_python_lidar({'lidar_name':'mid360','tcp_port':port,'stage_prim_path':'/test'})
    if tcp:
        assert len(node.publishers)==0 and all(len(s.frames)==1 for s in streams.values())
        for stream,typ in zip(streams.values(),(Imu,Image,PointCloud2)):
            msg=deserialize_message(stream.frames[0],typ)
            assert msg.header.stamp.nanosec==123_000_000
    else:
        assert len(node.publishers)==3 and all(len(p.messages)==1 for p in node.publishers)
        assert not any(s.frames for s in streams.values())
print('PUBLICATION_OK: actual IMU/RGB/XYZ methods use exactly one transport; CDR clock/payload valid')
