#include <zav_super/trajectory.hpp>
#include <ommpc_core/controller_core.hpp>
#include <iostream>
#include <stdexcept>
#include <limits>
void require(bool pass,const char *message){if(!pass)throw std::runtime_error(message);}
int main(){
  using namespace ommpc_core;
  mars_quadrotor_msgs::msg::PolynomialTrajectory m;
  m.order_pos=7;m.piece_num_pos=2;m.time_pos={2.,3.};
  m.coef_pos_x=std::vector<double>(16,0.);m.coef_pos_y=m.coef_pos_x;m.coef_pos_z=m.coef_pos_x;
  m.coef_pos_x[6]=1.;m.coef_pos_x[14]=1.;m.coef_pos_x[15]=2.;
  const auto trajectory=zav_super::decode(m);
  Trajectory stopped;
  Eigen::MatrixXd stop_coefficients=Eigen::MatrixXd::Zero(3,8);
  stop_coefficients.col(7)<<1.,2.,3.;
  stopped.emplace_back(2.,stop_coefficients);
  const Eigen::Vector3d endpoint(1.,2.,3.);
  require(zav_super::terminal_stop_reached(stopped,3.,endpoint,Eigen::Vector3d::Zero()),
          "Verified terminal stop cannot hold");
  require(!zav_super::terminal_stop_reached(stopped,1.9,endpoint,Eigen::Vector3d::Zero()),
          "Unfinished reference bypassed heartbeat protection");
  require(!zav_super::terminal_stop_reached(stopped,3.,endpoint+Eigen::Vector3d(.4,0,0),Eigen::Vector3d::Zero()),
          "Position excursion bypassed heartbeat protection");
  require(!zav_super::terminal_stop_reached(stopped,3.,endpoint,Eigen::Vector3d(.4,0,0)),
          "Moving vehicle bypassed heartbeat protection");
  require(!zav_super::terminal_stop_reached(trajectory,6.,trajectory.getPos(5.),Eigen::Vector3d::Zero()),
          "Nonzero terminal velocity accepted as a stop");
  Trajectory accelerating;
  Eigen::MatrixXd acceleration=Eigen::MatrixXd::Zero(3,8);
  acceleration(0,5)=1.;acceleration(0,6)=-4.;acceleration(0,7)=4.;
  accelerating.emplace_back(2.,acceleration);
  require(!zav_super::terminal_stop_reached(accelerating,3.,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()),
          "Nonzero terminal acceleration accepted as a stop");
  Trajectory changing_acceleration;
  Eigen::MatrixXd cubic=Eigen::MatrixXd::Zero(3,8);
  cubic(0,4)=1.;cubic(0,5)=-6.;cubic(0,6)=12.;cubic(0,7)=-8.;
  changing_acceleration.emplace_back(2.,cubic);
  require(!zav_super::terminal_stop_reached(changing_acceleration,3.,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()),
          "Nonzero terminal jerk accepted as a stop");
  require(!zav_super::terminal_stop_reached(stopped,3.,endpoint,Eigen::Vector3d(.2,.2,.2)),
          "Terminal hold speed must use the full velocity vector");
  require(!zav_super::terminal_stop_reached(stopped,3.,
          Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),0,0),Eigen::Vector3d::Zero()),
          "Nonfinite feedback accepted for terminal hold");
  require(!zav_super::terminal_stop_reached(Trajectory(),3.,endpoint,Eigen::Vector3d::Zero()),
          "Missing trajectory accepted for terminal hold");
  require(std::abs(trajectory.getPos(3.)[0]-3.)<1e-8,"Polynomial segment/order mismatch");
  m.coef_pos_x.pop_back();bool rejected=false;
  try{zav_super::decode(m);}catch(const std::invalid_argument &){rejected=true;}
  require(rejected,"Malformed trajectory accepted");
  const double hover=zav_super::thrust_command(9.81,1.99,5.4935e-6,1937.4,155.);
  const double speed=hover*1937.4+155.;
  require(std::abs(4*5.4935e-6*speed*speed/1.99-9.81)<1e-10,"Thrust inversion mismatch");
  require(zav_super::thrust_command(1000.,1.99,5.4935e-6,1937.4,155.)==1.,"Thrust saturation failed");
  ParameterSet p;p.mpc.Q_pos_xy=1000.;p.mpc.Q_pos_z=800.;p.mpc.Q_velocity=20.;
  p.mpc.Q_attitude_rp=p.mpc.Q_attitude_yaw=40.;p.mpc.R_thrust=.5;p.mpc.R_pitchroll=1.2;p.mpc.R_yaw=.6;
  p.mpc.state_cost_exponential=p.mpc.input_cost_exponential=.5;
  p.mpc.max_bodyrate_xy=4.;p.mpc.max_bodyrate_z=2.;p.mpc.min_thrust=1.;p.mpc.max_thrust=30.;
  OMMPCControllerCore controller;controller.init(p);
  ControllerState state;ControlCommand command;
  require(controller.setHoverReference(Eigen::Vector4d::Zero()),"Hover reference rejected");
  for(int i=0;i<200;i++){controller.setHoverReference(Eigen::Vector4d::Zero());require(controller.execMPC(state,i*.01,command),"Hover solver failed");}
  require(std::abs(command.collective_acceleration-9.81)<.01,"Incorrect hover thrust");
  require(command.bodyrates.norm()<1e-6,"Hover has nonzero angular rates");
  state.p.x()=-.2;
  controller.setHoverReference(Eigen::Vector4d::Zero());
  require(controller.execMPC(state,3.,command),"Offset hover solver failed");
  require(command.bodyrates.y()>0.,"Positive x correction has incorrect pitch sign");
  state.p.setZero();
  Trajectory position,yaw;
  Eigen::MatrixXd fixed=Eigen::MatrixXd::Zero(3,8);
  position.emplace_back(2.,fixed);fixed(0,7)=3.141592653589793;
  yaw.emplace_back(2.,fixed);
  require(controller.setTrajectoryReference(position,0.,0.,yaw,state),"Yaw-step reference failed");
  require(controller.execMPC(state,4.,command),"Yaw-step solve failed");
  require(command.bodyrates.head<2>().norm()<1e-5,"Yaw replan coupled into roll/pitch at hover");
  require(command.bodyrates.z()>0.,"Yaw replan has wrong turn direction");
  std::cout<<"PASS: SUPER segments, verified terminal stops, malformed input, nonlinear thrust, OMMPC hover and pitch response\n";
}
