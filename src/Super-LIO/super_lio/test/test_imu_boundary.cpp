#include <gtest/gtest.h>
#include "lio/imu_boundary.h"
using namespace LI2Sup;
using namespace BASIC;

TEST(ImuBoundary, CompletesScanWithoutConsumingFutureSample) {
  IMUData a, b;
  a.secs = .095; b.secs = .100;
  a.acc = V3(1, 0, 9.80665); b.acc = V3(2, 0, 9.80665);
  a.gyr = V3(0, 0, .1); b.gyr = V3(0, 0, .2);
  std::deque<IMUData> samples{a};
  ASSERT_TRUE(appendImuBoundary(samples, b, .0998));
  ASSERT_EQ(samples.size(), 2u);
  EXPECT_DOUBLE_EQ(samples.back().secs, .0998);
  EXPECT_NEAR(samples.back().acc.x(), 1.96, 1e-6);
  EXPECT_NEAR(samples.back().gyr.z(), .196, 1e-6);
  EXPECT_DOUBLE_EQ(b.secs, .100);
  EXPECT_FALSE(appendImuBoundary(samples, b, .0998));
  EXPECT_FALSE(appendImuBoundary(samples, b, .101));
}

TEST(ImuBoundary, DoesNotInventAnIntervalWithoutPredecessor) {
  std::deque<IMUData> samples;
  IMUData next; next.secs = 1.;
  EXPECT_FALSE(appendImuBoundary(samples, next, .9));
  EXPECT_TRUE(samples.empty());
}
