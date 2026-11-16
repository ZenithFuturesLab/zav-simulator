#pragma once
// High-rate output only: FAST-LIO's ESKF/map update remains upstream.
// Each scan correction resets this predictor, then replays buffered real IMU
// measurements to the newest measurement timestamp (never receive time).
namespace zav_prediction {
inline state_ikfom state;
inline bool ready=false;
inline double timestamp=0., last_published=0.;
inline std::deque<sensor_msgs::msg::Imu> history;
inline rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
inline V3D omega=V3D::Zero();
inline void integrate(const sensor_msgs::msg::Imu &imu) {
  const double t=get_time_sec(imu.header.stamp),dt=t-timestamp;
  if(dt<=0.)return;
  if(dt>.1){ready=false;return;}
  const auto &a=imu.linear_acceleration;const auto &w=imu.angular_velocity;
  omega=V3D(w.x,w.y,w.z)-state.bg;
  V3D force=V3D(a.x,a.y,a.z)-state.ba;
  const auto midpoint=state.rot*SO3(Exp(omega,dt*.5));
  const V3D acc=midpoint*force+V3D(state.grav[0],state.grav[1],state.grav[2]);
  state.pos+=state.vel*dt+.5*acc*dt*dt;state.vel+=acc*dt;
  state.rot=state.rot*SO3(Exp(omega,dt));timestamp=t;
}
inline void publish() {
  if(!ready || !publisher || timestamp<=last_published)return;
  nav_msgs::msg::Odometry m;m.header.stamp=get_ros_time(timestamp);
  m.header.frame_id="camera_init";m.child_frame_id="aft_mapped";
  m.pose.pose.position.x=state.pos.x();m.pose.pose.position.y=state.pos.y();m.pose.pose.position.z=state.pos.z();
  Eigen::Quaterniond q(state.rot.toRotationMatrix());
  m.pose.pose.orientation.w=q.w();m.pose.pose.orientation.x=q.x();m.pose.pose.orientation.y=q.y();m.pose.pose.orientation.z=q.z();
  const V3D v=state.rot.toRotationMatrix().transpose()*state.vel;
  m.twist.twist.linear.x=v.x();m.twist.twist.linear.y=v.y();m.twist.twist.linear.z=v.z();
  m.twist.twist.angular.x=omega.x();m.twist.twist.angular.y=omega.y();m.twist.twist.angular.z=omega.z();
  for(int i: {0,7,14}){m.twist.covariance[i]=.04;m.pose.covariance[i]=.04;}
  for(int i: {21,28,35})m.pose.covariance[i]=.03;
  publisher->publish(m);last_published=timestamp;
}
inline void imu(const sensor_msgs::msg::Imu &m) {
  history.push_back(m);
  while(!history.empty() && get_time_sec(m.header.stamp)-get_time_sec(history.front().header.stamp)>2.)history.pop_front();
  if(ready){integrate(m);publish();}
}
inline void correct(const state_ikfom &corrected,double t) {
  state=corrected;timestamp=t;ready=true;
  for(const auto &m:history)if(get_time_sec(m.header.stamp)>timestamp)integrate(m);
  publish();
}
}
