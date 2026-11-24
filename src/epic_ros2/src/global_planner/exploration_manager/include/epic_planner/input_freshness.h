#pragma once
#include <cmath>

namespace fast_planner {
// Compares timestamps in one clock domain, independent of message receipt.
// Startup requires several new measurements; a cached sample cannot qualify.
class InputFreshness {
 public:
  double max_age=.10,confirmation=.5;
  bool observe(double reference,double measurement,bool settle) {
    const bool valid=std::isfinite(reference) && std::isfinite(measurement) &&
        reference>=0. && measurement>=0. && reference-measurement>=-.03 &&
        reference-measurement<=max_age;
    if(!valid) {since_=-1.;count_=0;last_reference_=reference;return false;}
    if(since_<0. || reference<last_reference_ || reference-last_reference_>.75) {
      since_=reference;count_=0;last_measurement_=-1.;
    }
    last_reference_=reference;
    if(measurement>last_measurement_) {++count_;last_measurement_=measurement;}
    return !settle || (count_>=3 && reference-since_>=confirmation);
  }
 private:
  double since_=-1.,last_reference_=-1.,last_measurement_=-1.;
  unsigned count_=0;
};
}
