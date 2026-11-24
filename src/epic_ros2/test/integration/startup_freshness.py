"""ROS integration: delayed live odometry must not start external exploration.

Run in an unused ROS domain, with no simulator/controller in that domain.
An independent clock and synthetic registered room cloud are test inputs.
"""
from pathlib import Path
import argparse,json,os,signal,subprocess,time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,Path as NavPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2,PointField,Imu
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from ament_index_python.packages import get_package_share_directory

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--domain',type=int,default=139)
a=parser.parse_args();a.output.mkdir(parents=True,exist_ok=False)
if not 0<=a.domain<=232:parser.error('domain must be between 0 and 232')
os.environ['ROS_DOMAIN_ID']=str(a.domain);os.environ['ROS_LOCALHOST_ONLY']='1'
config=Path(get_package_share_directory('epic_ros2'))/'config/isaac_building_explore.yaml'
rclpy.init();node=rclpy.create_node('epic_startup_freshness_integration')
for _ in range(10):rclpy.spin_once(node,timeout_sec=.1)
peers=[name for name in node.get_node_names()if name!=node.get_name()]
if peers:raise RuntimeError('Integration test domain is occupied: '+str(peers))
clock=node.create_publisher(Clock,'/clock',qos_profile_sensor_data)
imu=node.create_publisher(Imu,'/test/startup/imu',qos_profile_sensor_data)
odom=node.create_publisher(Odometry,'/test/startup/odom',qos_profile_sensor_data)
cloud=node.create_publisher(PointCloud2,'/test/startup/cloud',qos_profile_sensor_data)
trigger=node.create_publisher(NavPath,'/epic/trigger',10)
received=[];states=[]
subs=[node.create_subscription(NavPath,'/mission/exploration_path',lambda m:received.append(m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
      node.create_subscription(Marker,'/epic/state',lambda m:states.append(m.text),10)]
points=[]
for x in np.arange(-8,8.01,.4):
 for z in np.arange(0,5.01,.4):
  points.extend([(x,-8,z),(x,8,z)])
for y in np.arange(-8,8.01,.4):
 for z in np.arange(0,5.01,.4):
  points.append((-8,y,z))
  if abs(y)>1.6:points.append((8,y,z))
for x in np.arange(-8,8.01,.4):
 for y in np.arange(-8,8.01,.4):points.extend([(x,y,0),(x,y,5)])
for x in np.arange(8,16.01,.4):
 for y in np.arange(-1.6,1.61,.4):points.extend([(x,y,0),(x,y,5)])
for y in np.arange(-1.6,1.61,.4):
 for z in np.arange(0,5.01,.4):points.append((16,y,z))
points=np.array(points,dtype='<f4')
log=(a.output/'navigation.log').open('w')
proc=subprocess.Popen(['ros2','launch','epic_ros2','isaac_epic.launch.py','external_local_planner:=true',f'config:={config}',
 'external_odometry_topic:=/test/startup/odom','external_cloud_topic:=/test/startup/cloud',
 'external_reference_imu_topic:=/test/startup/imu'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def stamp(value):
 from builtin_interfaces.msg import Time
 sec,nsec=divmod(int(round(value*1.e9)),1000000000);return Time(sec=sec,nanosec=nsec)
def send(t,delay):
 # Reproduce the circular-clock failure: adapter clock and odometry lag
 # together, while the source IMU reports the current measurement epoch.
 c=Clock();c.clock=stamp(t-delay);clock.publish(c)
 raw=Imu();raw.header.stamp=stamp(t);raw.header.frame_id='imu';imu.publish(raw)
 o=Odometry();o.header.frame_id='world';o.child_frame_id='zav_body';o.header.stamp=stamp(t-delay)
 o.pose.pose.position.z=1.6;o.pose.pose.orientation.w=1.;odom.publish(o)
 p=PointCloud2();p.header=o.header;p.height=1;p.width=len(points);p.point_step=12;p.row_step=12*len(points);p.is_dense=True
 p.fields=[PointField(name=k,offset=4*i,datatype=PointField.FLOAT32,count=1)for i,k in enumerate('xyz')];p.data=points.tobytes();cloud.publish(p)
 if states and states[-1]=='WAIT_TRIGGER':
  request=NavPath();request.header.frame_id='world';pose=PoseStamped();pose.pose.orientation.w=1.;request.poses=[pose];trigger.publish(request)
 rclpy.spin_once(node,timeout_sec=.02)
 time.sleep(.03)
try:
 deadline=time.monotonic()+30
 while odom.get_subscription_count()==0 and time.monotonic()<deadline:
  send(20.,.56)
 assert odom.get_subscription_count()>0,'EPIC live odometry subscription did not initialize'
 t=20.
 for _ in range(40):t+=.05;send(t,.56)
 assert not received,'EPIC published an exploration guide using delayed live odometry'
 stale_phase_end=t
 for _ in range(400):
  t+=.05;send(t,0.)
  if received:break
  if proc.poll()is not None:raise RuntimeError('EPIC process exited')
 assert received,'Fresh state did not release external exploration'
 first_guide_time=t
 before_quiet=len(received)
 for _ in range(35):t+=.05;send(t,0.)
 assert len(received)==before_quiet,'Short startup preview caused guide churn during initial handoff'
 for _ in range(200):
  t+=.05;send(t,0.)
  if len(received)>before_quiet:break
 assert len(received)>before_quiet,'Guide renewal did not resume after the initial handoff'
 result={'delayed_measurement_s':.56,'guides_while_delayed':0,'guides_after_fresh':len(received),
         'adapter_clock_and_odometry_delayed_together':True,'independent_reference':'raw IMU timestamp',
         'first_guide_after_fresh_s':round(first_guide_time-stale_phase_end,3),
         'guides_during_first_1_75s':before_quiet,'renewal_resumed_after_sim_s':round(t-first_guide_time,3),
         'states':list(dict.fromkeys(states)),
         'domain':os.environ.get('ROS_DOMAIN_ID','0')}
 (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
finally:
 if proc.poll()is None:os.killpg(proc.pid,signal.SIGINT)
 try:proc.wait(timeout=20)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
 log.close();node.destroy_node();rclpy.shutdown()
