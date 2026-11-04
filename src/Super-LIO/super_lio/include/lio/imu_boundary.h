#pragma once

#include <cmath>
#include <deque>
#include "common/ds.h"

namespace LI2Sup {

// Complete the integration interval without consuming the first IMU sample of
// the next scan. Never extrapolate beyond the samples actually received.
inline bool appendImuBoundary(std::deque<IMUData>& samples,
                              const IMUData& next, double end_time) {
  if (samples.empty()) return false;
  const auto& previous = samples.back();
  if (!std::isfinite(end_time) || !std::isfinite(previous.secs) ||
      !std::isfinite(next.secs) || !(previous.secs < end_time) ||
      !(end_time <= next.secs)) return false;
  const double alpha = (end_time - previous.secs) / (next.secs - previous.secs);
  IMUData boundary = previous;
  boundary.secs = end_time;
  boundary.acc += alpha * (next.acc - previous.acc);
  boundary.gyr += alpha * (next.gyr - previous.gyr);
  samples.push_back(boundary);
  return true;
}

}  // namespace LI2Sup
