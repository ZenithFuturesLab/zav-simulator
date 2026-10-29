#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include "ros/odometry_compat.h"

TEST(OdometryCompat, VelocityIsInChildFrameAndPoseStampArePreserved) {
  nav_msgs::msg::Odometry odom;
  odom.header.stamp.sec = 12;
  odom.pose.pose.position.x = 3.;
  odom.twist.twist.angular.z = .2;
  const Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d world_velocity(1.,0.,0.);
  LI2Sup::setFastLivoContract(odom,R,world_velocity);
  EXPECT_EQ(odom.header.frame_id,"camera_init");
  EXPECT_EQ(odom.child_frame_id,"aft_mapped");
  EXPECT_NEAR(odom.twist.twist.linear.x,0.,1e-10);
  EXPECT_NEAR(odom.twist.twist.linear.y,-1.,1e-10);
  EXPECT_DOUBLE_EQ(odom.twist.twist.linear.z,0.);
  EXPECT_EQ(odom.header.stamp.sec,12);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.x,3.);
  EXPECT_DOUBLE_EQ(odom.twist.twist.angular.z,.2);
  for(int i=0;i<3;++i) EXPECT_DOUBLE_EQ(odom.twist.covariance[i*7],.04);
}

TEST(OdometryCompat, MountedImuVelocityRoundTrip) {
  nav_msgs::msg::Odometry odom;
  const Eigen::Matrix3f R = (Eigen::AngleAxisf(.7f,Eigen::Vector3f::UnitZ()) *
                             Eigen::AngleAxisf(.384f,Eigen::Vector3f::UnitY())).toRotationMatrix();
  const Eigen::Vector3f world_velocity(.8f,-.3f,.2f);
  LI2Sup::setFastLivoContract(odom,R,world_velocity);
  const Eigen::Vector3f child(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z);
  EXPECT_LT((R*child-world_velocity).norm(),1e-6);
}
