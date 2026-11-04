#include <gtest/gtest.h>
#include "IMU_Processing.h"

static sensor_msgs::msg::Imu::ConstPtr imuAt(int tick, double yaw_rate) {
  auto imu = std::make_shared<sensor_msgs::msg::Imu>();
  imu->header.stamp.nanosec = tick * 5000000;
  imu->linear_acceleration.z = G_m_s2;
  imu->angular_velocity.z = yaw_rate;
  return imu;
}

TEST(SparseDeskew, UsesPhysicalPointTimeAcrossScanGapIncludingFirstPoint) {
  ImuProcess processor;
  processor.first_lidar_time = 0.;
  processor.set_imu_init_frame_num(0);
  StatesGroup state;
  LidarMeasureGroup group;
  group.lio_vio_flg = LIO;
  MeasureGroup initial;
  initial.lio_time = .005;
  initial.imu = {imuAt(0, 0.), imuAt(1, 0.)};
  group.measures.push_back(initial);
  PointCloudXYZI::Ptr output(new PointCloudXYZI());
  processor.Process2(group, state, output);
  ASSERT_FALSE(processor.imu_need_init);
  group.measures.clear();
  group.lidar_frame_beg_time = .055;
  group.lidar_frame_end_time = .105;
  MeasureGroup measurement;
  measurement.lio_time = .105;
  for (int tick=2; tick<=21; ++tick) measurement.imu.push_back(imuAt(tick, 1.));
  group.measures.push_back(measurement);
  const V3D world_point(2., .3, .1);
  for (double offset : {0., 0., 50., 50.}) {
    const double angle = .0475 + offset / 1000.;
    const V3D local = Eigen::AngleAxisd(-angle, V3D::UnitZ()) * world_point;
    PointType point;
    point.x=local.x(); point.y=local.y(); point.z=local.z(); point.curvature=offset;
    group.lidar->push_back(point);
  }
  group.pcl_proc_cur = group.lidar;
  processor.Process2(group, state, output);
  ASSERT_EQ(output->size(), 4u);
  const V3D expected = Eigen::AngleAxisd(-.0975, V3D::UnitZ()) * world_point;
  for (const auto& point : output->points) {
    EXPECT_LT((V3D(point.x, point.y, point.z)-expected).norm(), 2e-5);
  }
}
