#include <gtest/gtest.h>
#include "preprocess.h"

TEST(LivoxTiming, PreservesSparseSnapshotTimes) {
  Preprocess processor;
  processor.set(false, AVIA, .5, 1);
  processor.blind_sqr = .25;
  auto message = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
  const uint32_t times[] = {0, 0, 50000000, 50000000, 100000000};
  for (const auto time : times) {
    livox_ros_driver2::msg::CustomPoint point;
    point.x = 2.; point.y = .1; point.z = .2;
    point.line = 0; point.tag = 0x10; point.offset_time = time;
    message->points.push_back(point);
  }
  message->point_num = message->points.size();
  PointCloudXYZI::Ptr output(new PointCloudXYZI());
  processor.process(message, output);
  ASSERT_EQ(output->size(), 5u);
  for (size_t i=0; i<output->size(); ++i) {
    EXPECT_NEAR(output->points[i].curvature, times[i] / 1000000., 1e-5);
  }
}
