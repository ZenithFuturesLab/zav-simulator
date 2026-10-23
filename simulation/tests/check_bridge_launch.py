"""Isolated ROS-domain launch and bridge contract verification; no PX4 needed."""
import json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from px4_msgs.msg import VehicleOdometry,SensorCombined,TimesyncStatus
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
os.environ['ROS_DOMAIN_ID']='49'
checkout=Path(__file__).resolve().parents[2]
root=checkout/'.cache/bridge-test';root.mkdir(parents=True,exist_ok=True)
log=open(root/'launch-smoke.log','w')
process=subprocess.Popen(['bash',str(checkout/'run_fastlivo.sh')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();n=rclpy.create_node('bridge_integration_verifier');q=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
pubs=[n.create_publisher(t,topic,q) for t,topic in [(SensorCombined,'/fmu/out/sensor_combined'),(TimesyncStatus,'/fmu/out/timesync_status'),(Imu,'/drone0/mid360/imu'),(Odometry,'/LIVO2/imu_propagate')]]
received=[];sub=n.create_subscription(VehicleOdometry,'/fmu/in/vehicle_visual_odometry',received.append,q)
def spin(t):
 end=time.monotonic()+t
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
try:
 deadline=time.monotonic()+15
 while time.monotonic()<deadline:
  spin(.1)
  if 'zav_lio_px4_odom_bridge' in n.get_node_names():break
 assert 'zav_lio_px4_odom_bridge' in n.get_node_names(),(root/'launch-smoke.log').read_text()
 spin(1)
 for i in range(80):
  sim_us=10_000_000+i*20_000;wall_us=time.time_ns()//1000
  px=SensorCombined();px.timestamp=wall_us;pubs[0].publish(px)
  ts=TimesyncStatus();ts.timestamp=wall_us;ts.estimated_offset=sim_us-wall_us;pubs[1].publish(ts)
  imu=Imu();imu.header.frame_id='mid360_imu';imu.header.stamp.sec,ns=divmod(sim_us*1000,10**9);imu.header.stamp.nanosec=ns;pubs[2].publish(imu);spin(.015)
  od=Odometry();od.header.frame_id='camera_init';od.child_frame_id='aft_mapped';od.header.stamp=imu.header.stamp
  od.pose.pose.orientation.w=math.cos(math.radians(11));od.pose.pose.orientation.y=math.sin(math.radians(11))
  od.twist.twist.linear.x=.3
  for k in (0,7,14):od.twist.covariance[k]=.04
  pubs[3].publish(od);spin(.02)
 assert len(received)>5,len(received)
 msg=received[-1];assert msg.pose_frame==VehicleOdometry.POSE_FRAME_FRD
 assert msg.velocity_frame==VehicleOdometry.VELOCITY_FRAME_FRD
 assert all(math.isfinite(x) for x in list(msg.position)+list(msg.velocity)+list(msg.q))
 assert abs(msg.q[1])<1e-5 and abs(msg.q[2])<1e-5,'mounted IMU rotation changed'
 assert 0<=msg.timestamp-msg.timestamp_sample<100_000
 duplicate=subprocess.run(['ros2','run','zav_control','lio_px4_bridge','--publish','--assume-enu-flu'],capture_output=True,text=True,timeout=5)
 assert duplicate.returncode!=0 and 'already running' in duplicate.stderr,duplicate.stderr
 result={'received_odometry_messages':len(received),'pose_frame':msg.pose_frame,'velocity_frame':msg.velocity_frame,'duplicate_publisher_blocked':True}
finally:
 process.send_signal(signal.SIGINT)
 try:process.wait(timeout=15)
 except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
 deadline=time.monotonic()+8
 while time.monotonic()<deadline:
  spin(.1)
  if 'zav_lio_px4_odom_bridge' not in n.get_node_names():break
 stopped='zav_lio_px4_odom_bridge' not in n.get_node_names()
 n.destroy_node();rclpy.shutdown();log.close()
assert stopped,'bridge did not stop with launcher'
result['stopped_with_launcher']=stopped
(root/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
