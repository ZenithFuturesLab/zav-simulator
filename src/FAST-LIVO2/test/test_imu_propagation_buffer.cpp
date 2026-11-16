#include <imu_propagation_buffer.hpp>
#include <vector>
#include <stdexcept>
#include <iostream>
struct Sample { double stamp, acceleration; };
int main() {
  try {
    std::vector<Sample> buffer;
    for(int i=1;i<=20;++i) buffer.push_back({i*.005,i==20 ? 150. : 0.});
    double time=0., position=0., velocity=0.;
    auto integrate=[&](const Sample &s,double dt) { position+=velocity*dt+.5*s.acceleration*dt*dt; velocity+=s.acceleration*dt; };
    auto stamp=[](const Sample &s) { return s.stamp; };
    if (!fast_livo::integrate_buffered_imu(buffer,time,stamp,integrate) ||
        std::abs(velocity-.75)>1e-9 || std::abs(position-.001875)>1e-9)
      throw std::runtime_error("batched impulse was held over the whole backlog");
    if (fast_livo::integrate_buffered_imu(buffer,time,stamp,integrate))
      throw std::runtime_error("buffer replayed without EKF reset");
    // A newer EKF baseline replays only its subsequent samples.
    time=.05; position=velocity=0.;
    fast_livo::integrate_buffered_imu(buffer,time,stamp,integrate);
    if (std::abs(velocity-.75)>1e-9) throw std::runtime_error("EKF reset replay wrong");
    time=.2;
    if (fast_livo::integrate_buffered_imu(buffer,time,stamp,integrate))
      throw std::runtime_error("published a state from before the EKF epoch");
    std::cout<<"batched IMU impulse and EKF replay tests passed\n";
    return 0;
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
