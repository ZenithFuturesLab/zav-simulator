// ZAV: FAST-LIVO mounted-IMU Odometry contract, opt-in in ROSWrapper.
#pragma once
#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>

namespace LI2Sup {
template<class Rotation, class Velocity>
inline void setFastLivoContract(nav_msgs::msg::Odometry& odom,
                                const Rotation& world_from_imu,
                                const Velocity& velocity_world) {
  odom.header.frame_id = "camera_init";
  odom.child_frame_id = "aft_mapped";
  const auto velocity_imu = (world_from_imu.transpose() * velocity_world).eval();
  odom.twist.twist.linear.x = velocity_imu[0];
  odom.twist.twist.linear.y = velocity_imu[1];
  odom.twist.twist.linear.z = velocity_imu[2];
  // Same conservative velocity contract as ZAV's patched FAST-LIVO output.
  // This is an assumed variance, not the propagated ESKF covariance.
  for (int axis=0; axis<3; ++axis) odom.twist.covariance[axis*7] = 0.04;
}
}
