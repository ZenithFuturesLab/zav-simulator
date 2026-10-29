#include <gtest/gtest.h>
#include "lio/ESKF.h"
using namespace LI2Sup;
using namespace BASIC;

TEST(ImuRepropagation, DelayedScanWithoutCorrectionDoesNotInventRotation) {
  g_gravity_norm=9.80665;
  ESKF::Options options;
  ESKF filter(options);
  filter.SetInitialConditions(options,V3::Zero(),V3::Zero(),1.f,V3(0,0,-g_gravity_norm));
  SysState initial;initial.timestamp=0.;filter.SetX(initial);filter.init_=true;
  std::vector<IMUData> samples;
  // Smooth angular-rate reversal, at the actual 200 Hz sensor cadence.
  for(int i=0;i<=41;++i){
    IMUData m;m.secs=i*.005;m.acc=V3(0,0,g_gravity_norm);
    m.gyr=V3(0,4.-40.*m.secs,0);samples.push_back(m);
  }
  filter.SetObsTime(.1);
  for(int i=0;i<=20;++i)filter.Predict(samples[i]);
  DynamicState current,robot;
  for(int i=0;i<=40;++i)filter.Predict(samples[i],current,robot);
  const M3 before=current.R;
  // Zero residual and zero observation information: no physical pose correction.
  filter.UpdateObserve([](const ESKF::KFState&,M6& H,V6& r){H.setZero();r.setZero();});
  ASSERT_TRUE(filter.Predict(samples[41],current,robot));
  const double change=SO3(before.transpose()*current.R).log_vee().norm();
  EXPECT_NEAR(change,4.1*.005,2e-5); // Trapezoidal gyro integral: 1.1746 degrees.
  EXPECT_FALSE(filter.Predict(samples[41],current,robot)); // duplicate measurement
}
