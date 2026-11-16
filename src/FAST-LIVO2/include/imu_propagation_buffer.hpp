#pragma once
#include <cmath>

namespace fast_livo {
// Integrate every received IMU sample at its own interval. Holding the newest
// acceleration over a callback backlog amplifies short collision impulses.
template <typename Buffer, typename Stamp, typename Integrate>
bool integrate_buffered_imu(const Buffer &buffer, double &last_time,
                            Stamp stamp, Integrate integrate) {
  bool advanced=false;
  for (const auto &sample : buffer) {
    const double t=stamp(sample);
    if (!std::isfinite(t) || t<=last_time) continue;
    integrate(sample,t-last_time);
    last_time=t;
    advanced=true;
  }
  return advanced;
}
}
