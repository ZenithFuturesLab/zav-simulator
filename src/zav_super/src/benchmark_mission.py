#!/usr/bin/env python3
"""Original SUPER benchmark waypoints, gated on OMMPC's completed takeoff."""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class Mission(Node):
    def __init__(self):
        super().__init__('zav_super_benchmark')
        self.declare_parameter('execute', False)
        self.declare_parameter('repetitions', 1)
        self.declare_parameter('reach_radius', .8)
        self.goals=[(0.,50.,1.5),(0.,-50.,1.5)]*self.get_parameter('repetitions').value
        self.index=0;self.sent=False;self.ready=False;self.started=False;self.future=None
        self.position=None;self.velocity=None;self.rx=0.;self.status='';self.last_goal=0.
        self.publisher=self.create_publisher(PoseStamped,'/planning/click_goal',10)
        self.markers=self.create_publisher(MarkerArray,'/mission/mkr',10)
        self.start=self.create_client(Trigger,'/zav/super/start')
        self.land=self.create_client(Trigger,'/zav/super/land')
        self.create_subscription(Odometry,'/zav/super/odom',self.odom,10)
        self.create_subscription(String,'/zav/super/controller_status',self.state,10)
        self.create_subscription(PoseStamped,'/goal_pose',self.click,10)
        self.create_timer(.2,self.tick)

    def odom(self,msg):
        self.position=msg.pose.pose.position;self.velocity=msg.twist.twist.linear;self.rx=time.monotonic()

    def state(self,msg): self.status=msg.data

    def click(self,msg):
        # Interactive goals share the same frame as the planner.
        if msg.header.frame_id=='world':
            self.publisher.publish(msg)

    def tick(self):
        if self.position is None or time.monotonic()-self.rx>1.: return
        if not self.get_parameter('execute').value: return
        if not self.started:
            if self.future is None and self.start.service_is_ready(): self.future=self.start.call_async(Trigger.Request())
            elif self.future is not None and self.future.done():
                self.started=self.future.result().success;self.future=None
            return
        if self.status not in ('READY','TRACKING'): return
        if self.index>=len(self.goals):
            if self.land.service_is_ready():
                self.land.call_async(Trigger.Request());self.get_logger().info('Benchmark complete; OMMPC landing requested')
                self.set_parameters([rclpy.parameter.Parameter('execute',value=False)])
            return
        x,y,z=self.goals[self.index]
        if not self.sent:
            msg=PoseStamped();msg.header.frame_id='world';msg.header.stamp=self.get_clock().now().to_msg()
            msg.pose.position.x=x;msg.pose.position.y=y;msg.pose.position.z=z;msg.pose.orientation.w=1.
            self.publisher.publish(msg);self.sent=True;self.last_goal=time.monotonic()
            self.get_logger().info(f'Waypoint {self.index+1}/{len(self.goals)}: {(x,y,z)}')
            markers=MarkerArray()
            for i,goal in enumerate(self.goals):
                marker=Marker();marker.header=msg.header;marker.ns='benchmark';marker.id=i;marker.type=Marker.SPHERE
                marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=goal;marker.pose.orientation.w=1.
                marker.scale.x=marker.scale.y=marker.scale.z=.4
                marker.color.g=1.;marker.color.a=.8;markers.markers.append(marker)
            self.markers.publish(markers)
        if math.dist((self.position.x,self.position.y,self.position.z),(x,y,z))<self.get_parameter('reach_radius').value:
            if math.sqrt(self.velocity.x**2+self.velocity.y**2+self.velocity.z**2)<.7:
                self.index+=1;self.sent=False


def main():
    rclpy.init();node=Mission()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
