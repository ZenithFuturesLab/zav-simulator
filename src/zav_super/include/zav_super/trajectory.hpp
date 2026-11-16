#pragma once
#include <mars_quadrotor_msgs/msg/polynomial_trajectory.hpp>
#include <ommpc_core/polynomial_trajectory.h>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace zav_super {
// SUPER stops sending commands while waiting after a committed stop. This is
// distinct from losing planner feedback during an unfinished/moving reference.
inline bool terminal_stop_reached(const Trajectory &trajectory, double elapsed,
                                 const Eigen::Vector3d &position,
                                 const Eigen::Vector3d &velocity) {
  if (trajectory.getPieceNum()==0 || !std::isfinite(elapsed) ||
      !position.allFinite() || !velocity.allFinite()) return false;
  const double end=trajectory.getTotalDuration();
  if (!std::isfinite(end) || end<=0. || elapsed<end) return false;
  const auto p=trajectory.getPos(end), v=trajectory.getVel(end);
  const auto a=trajectory.getAcc(end), j=trajectory.getJer(end);
  return p.allFinite() && v.allFinite() && a.allFinite() && j.allFinite() &&
         v.norm()<1e-3 && a.norm()<1e-3 && j.norm()<1e-3 &&
         (position-p).norm()<=.3 && velocity.norm()<=.3;
}

inline Trajectory decode(const mars_quadrotor_msgs::msg::PolynomialTrajectory &m, bool yaw=false) {
  const auto &times=yaw?m.time_yaw:m.time_pos;
  const auto order=yaw?m.order_yaw:m.order_pos;
  const auto count=yaw?m.piece_num_yaw:m.piece_num_pos;
  if (count==0 || count>1000 || order!=7 || times.size()!=count)
    throw std::invalid_argument("Invalid SUPER polynomial dimensions/order");
  const size_t size=count*(order+1);
  if ((yaw && m.coef_yaw.size()!=size) || (!yaw &&
      (m.coef_pos_x.size()!=size || m.coef_pos_y.size()!=size || m.coef_pos_z.size()!=size)))
    throw std::invalid_argument("Truncated SUPER coefficients");
  Trajectory result;
  for (size_t i=0;i<count;++i) {
    if (!std::isfinite(times[i]) || times[i]<=0.) throw std::invalid_argument("Invalid segment duration");
    Eigen::MatrixXd coefficients=Eigen::MatrixXd::Zero(3,8);
    for (size_t j=0;j<8;++j) {
      coefficients(0,j)=yaw?m.coef_yaw[i*8+j]:m.coef_pos_x[i*8+j];
      if (!yaw) {coefficients(1,j)=m.coef_pos_y[i*8+j];coefficients(2,j)=m.coef_pos_z[i*8+j];}
    }
    if (!coefficients.allFinite()) throw std::invalid_argument("Nonfinite coefficients");
    result.emplace_back(times[i],coefficients);
  }
  return result;
}
// Simulator input is angular velocity = command * scale + armed idle.
// PX4 THR_MDL_FAC=0: actuator value is passed linearly, so the net collective
// acceleration is quadratic. Invert that model, instead of a linear hover ratio.
inline double thrust_command(double acceleration, double mass, double coefficient,
                             double scale, double idle) {
  if (!std::isfinite(acceleration) || acceleration<0) throw std::invalid_argument("Invalid thrust");
  return std::clamp((std::sqrt(mass*acceleration/(4.*coefficient))-idle)/scale,0.,1.);
}
}
