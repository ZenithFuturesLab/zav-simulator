#!/usr/bin/env python3
"""Isolated deterministic raw-CDR replay + odometry/CPU collection for one mapper."""
import argparse,json,os,signal,sqlite3,struct,subprocess,time
from pathlib import Path
import numpy as np
import psutil
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg

ROOT=Path(__file__).resolve().parents[2]
P=ROOT/'.cache/lio-fixed-20261102'
parser=argparse.ArgumentParser();parser.add_argument('--algorithm',choices=['fast_livo','super_lio'],required=True);parser.add_argument('--profile',choices=['common'],required=True);parser.add_argument('--trial',type=int,required=True);a=parser.parse_args()
output=ROOT/'reports/lio-fixed-20261102/raw'/f'{a.profile}-{a.algorithm}-{a.trial}';output.mkdir(parents=True,exist_ok=False)
os.sched_setaffinity(0,{4,5})

def stamp(raw):
 sec,ns=struct.unpack_from('<iI',raw,4);return sec*10**9+ns

def scan_info(raw):
 # CDR offsets are aligned relative to the four-byte encapsulation header.
 n,=struct.unpack_from('<I',raw,12);pos=16+n
 pos=4+((pos-4+7)//8)*8
 count,=struct.unpack_from('<I',raw,pos+16)
 offsets=np.ndarray(shape=(count,),dtype='<u4',buffer=raw,offset=pos+20,strides=(20,))
 return int(offsets.max(initial=0)),count

conn=sqlite3.connect(P/'common.db3')
lookup={x[0]:x[1] for x in conn.execute('select id,name from topics')}
events=[];counts={'imu':0,'lidar':0};points=[];last_observed_imu=0;delay_rows=[]
for topic_id,raw in conn.execute('select topic_id,data from messages order by id'):
 name=lookup[topic_id];t=stamp(raw)
 if name.endswith('/lidar'):
  dt,n=scan_info(raw);t+=dt;acquired=t;t=max(t,last_observed_imu);delay_rows.append(dict(acquired_ns=acquired,available_ns=t,delay_ms=(t-acquired)/1e6));priority=1;counts['lidar']+=1;points.append(n)
 else:last_observed_imu=max(last_observed_imu,t);priority=0;counts['imu']+=1
 events.append((t,priority,name,bytes(raw)))
flight=[json.loads(s) for s in (P/'capture.jsonl').read_text().splitlines()]
cutoff=min(x['truth']['sim'] for x in flight if x['stage']=='hover_final')+.05
events=[x for x in events if x[0]*1e-9<=cutoff]
(output/'input_availability.json').write_text(json.dumps(delay_rows,indent=2))
events.sort(key=lambda x:(x[0],x[1]));first=events[0][0];last=events[-1][0]
counts={'imu':sum(x[1]==0 for x in events),'lidar':sum(x[1]==1 for x in events)}
# Validate the direct CDR point-array parser against ROS deserialization once.
from rclpy.serialization import deserialize_message
raw=next(x[3] for x in events if x[1]==1);msg=deserialize_message(raw,CustomMsg)
assert scan_info(raw)==(max(p.offset_time for p in msg.points),len(msg.points))

rclpy.init();node=rclpy.create_node('lio_benchmark_replay')
q=QoSProfile(depth=2000,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
pubs={'/drone0/mid360/imu':node.create_publisher(Imu,'/drone0/mid360/imu',q),'/drone0/mid360/lidar':node.create_publisher(CustomMsg,'/drone0/mid360/lidar',q)}
odom=(output/'odometry.jsonl').open('w',buffering=1)
def collect(label,m):
 p=m.pose.pose.position;qq=m.pose.pose.orientation;v=m.twist.twist.linear
 odom.write(json.dumps(dict(kind=label,t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,wall=time.monotonic(),p=[p.x,p.y,p.z],q=[qq.x,qq.y,qq.z,qq.w],v=[v.x,v.y,v.z]))+'\n')
subs=[node.create_subscription(Odometry,'/aft_mapped_to_init',lambda m:collect('lidar',m),q),node.create_subscription(Odometry,'/LIVO2/imu_propagate',lambda m:collect('imu',m),q)]
binary=P/'install'/a.algorithm/'lib'/a.algorithm/('fastlivo_mapping' if a.algorithm=='fast_livo' else 'super_lio_node')
cmd=['taskset','-c','0,1,2,3',str(binary),'--ros-args','-r','__node:='+('laserMapping' if a.algorithm=='fast_livo' else 'super_lio'),'--params-file',str(P/f'{a.algorithm}-{a.profile}.yaml')]
if a.algorithm=='fast_livo':cmd+=['--params-file',str(ROOT/'src/FAST-LIVO2/config/zenith_camera_pinhole.yaml')]
else:
 for x,y in [('/lio/odom','/aft_mapped_to_init'),('/lio/imu/odom','/LIVO2/imu_propagate'),('/lio/cloud_world','/cloud_registered'),('/lio/path','/path')]:cmd+=['-r',x+':='+y]
env=dict(os.environ);env['ZAV_COMMON_POINTS']='1';env['CYCLONEDDS_URI']=(ROOT/'simulator/tools/rviz/cyclonedds_fastlivo.xml').as_uri();env['ZAV_BENCH_CSV']=str(output/'stages.csv');env.update(OMP_NUM_THREADS='4',OPENBLAS_NUM_THREADS='1',OMP_WAIT_POLICY='PASSIVE')
log=(output/'mapper.log').open('w');proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT);mon=psutil.Process(proc.pid)
cpu=(output/'resources.jsonl').open('w',buffering=1)
try:
 deadline=time.monotonic()+20
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.05)
  if proc.poll() is not None:raise RuntimeError('Mapper exited before replay')
  if all(p.get_subscription_count()>0 for p in pubs.values()):break
 else:raise RuntimeError('Sensor subscription discovery timeout')
 # Allow odometry subscriptions to discover before the first initialized output.
 discovery_end=time.monotonic()+1
 while time.monotonic()<discovery_end:rclpy.spin_once(node,timeout_sec=.01)
 start=time.monotonic();i=0;sample_next=start;last_sim=first*1e-9
 while i<len(events):
  now=time.monotonic()
  while i<len(events) and (events[i][0]-first)*1e-9<=now-start:
   t,_,topic,raw=events[i];pubs[topic].publish(raw);last_sim=t*1e-9;i+=1
  for _ in range(6):rclpy.spin_once(node,timeout_sec=0)
  now=time.monotonic()
  if now>=sample_next:
   c=mon.cpu_times();mem=mon.memory_info()
   cpu.write(json.dumps(dict(wall=now-start,sim=last_sim,cpu_user=c.user,cpu_system=c.system,rss=mem.rss,threads=mon.num_threads(),host_cpu_percent=psutil.cpu_percent()))+'\n');sample_next=now+.1
  if proc.poll() is not None:raise RuntimeError('Mapper exited during replay')
  if i<len(events):time.sleep(max(0.,min(.001,(events[i][0]-first)*1e-9-(time.monotonic()-start))))
 end=time.monotonic()+3
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.005)
 result=dict(algorithm=a.algorithm,profile=a.profile,trial=a.trial,input_counts=counts,input_points_mean=float(np.mean([scan_info(x[3])[1] for x in events if x[1]==1])),first_stamp=first*1e-9,last_stamp=last*1e-9,sim_duration=(last-first)*1e-9,replay_rate=1.,command=cmd)
 (output/'run.json').write_text(json.dumps(result,indent=2))
finally:
 if proc.poll() is None:
  proc.send_signal(signal.SIGINT)
  try:proc.wait(timeout=8)
  except subprocess.TimeoutExpired:proc.kill();proc.wait()
 cpu.close();odom.close();log.close();node.destroy_node();rclpy.shutdown()
print('COMPLETE',output,flush=True)
