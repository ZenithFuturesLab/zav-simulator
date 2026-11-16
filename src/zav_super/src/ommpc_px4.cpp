#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <ommpc_core/controller_core.hpp>
#include <zav_super/trajectory.hpp>
#include <chrono>
#include <algorithm>
#include <sys/file.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;
using Poly=mars_quadrotor_msgs::msg::PolynomialTrajectory;
using Steady=std::chrono::steady_clock;

class Controller : public rclcpp::Node {
  ommpc_core::OMMPCControllerCore mpc_;
  ommpc_core::ControllerState state_;
  Trajectory trajectory_,yaw_;
  Eigen::Vector4d hold_=Eigen::Vector4d::Zero();
  double touchdown_start_=-1.;
  double start_=0., last_sim_=0., last_step_=0., yaw_start_=0.,ground_=0.;
  bool allow_terminal_hold_=false;
  bool use_px4_landing_=false, px4_land_started_=false;
  bool odom_ok_=false, active_=false, requested_=false, flight_=false, landing_=false, fault_=false;
  bool landed_=true, entered_offboard_=false, safety_latched_=false;
  int lock_fd_=-1, failures_=0;
  uint8_t nav_=0, arming_=0;
  uint64_t px4_stamp_=0;
  Steady::time_point odom_rx_{}, px4_rx_{}, begin_{}, last_command_{}, heartbeat_rx_{};
  Steady::time_point vehicle_rx_{}, land_rx_{}, land_requested_at_{};
  double control_period_=0.01, step_=0.01, mass_=1.99, thrust_k_=5.4935e-6, scale_=1937.4, idle_=155.;
  double takeoff_height_=1.5, climb_speed_=.4;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr commands_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<Poly>::SharedPtr traj_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr time_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_, land_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  double elapsed(Steady::time_point t) const {return std::chrono::duration<double>(Steady::now()-t).count();}
  uint64_t stamp() const {return px4_stamp_+static_cast<uint64_t>(elapsed(px4_rx_)*1e6);}
  void report(const std::string &text) {std_msgs::msg::String m;m.data=text;status_->publish(m);}
  void command(uint16_t cmd, float p1=0.,float p2=0.) {
    px4_msgs::msg::VehicleCommand m; m.timestamp=stamp();m.command=cmd;m.param1=p1;m.param2=p2;
    m.target_system=1;m.target_component=1;m.source_system=245;m.source_component=191;m.from_external=true;
    commands_->publish(m);last_command_=Steady::now();
  }
  void fail(const std::string &reason) {
    if (!fault_) {
      RCLCPP_ERROR(get_logger(),"%s; requesting PX4 land and ending OFFBOARD stream",reason.c_str());
      command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
      report("FAULT: "+reason);
    }
    fault_=true;active_=false;requested_=false;
  }
  void odom(const nav_msgs::msg::Odometry &m) {
    if (m.header.frame_id!="world" || m.child_frame_id!="zav_body") return;
    const auto &p=m.pose.pose.position;const auto &q=m.pose.pose.orientation;
    const auto &v=m.twist.twist.linear;const auto &w=m.twist.twist.angular;
    ommpc_core::ControllerState candidate;
    candidate.p={p.x,p.y,p.z};candidate.q={q.w,q.x,q.y,q.z};
    candidate.v={v.x,v.y,v.z};candidate.w={w.x,w.y,w.z};
    candidate.timestamp_s=rclcpp::Time(m.header.stamp).seconds();
    if (!candidate.p.allFinite() || !candidate.v.allFinite() || !candidate.w.allFinite() ||
        !candidate.q.coeffs().allFinite() || candidate.q.norm()<.9 || candidate.timestamp_s<=last_sim_) return;
    if (odom_ok_ && (candidate.p-state_.p).norm()> .5+35.*(candidate.timestamp_s-last_sim_)) {
      if (active_) fail("LIO position discontinuity"); return;
    }
    state_=candidate;state_.q.normalize();last_sim_=candidate.timestamp_s;
    odom_ok_=true;odom_rx_=Steady::now();
  }
  void trajectory(const Poly &m) {
    if (m.header.frame_id!="world" || !flight_ || landing_ || fault_) return;
    if (m.type & Poly::EMER_STOP) {fail("SUPER emergency stop");return;}
    if (m.type & Poly::HEART_BEAT) heartbeat_rx_=Steady::now();
    if (!(m.type & Poly::POSITION_TRAJ)) return;
    try {
      auto p=zav_super::decode(m);
      Trajectory y;
      if (m.type & Poly::YAW_TRAJ) y=zav_super::decode(m,true);
      if (!std::isfinite(m.start_wt_pos) || m.start_wt_pos>last_sim_+1. ||
          m.start_wt_pos+p.getTotalDuration()<last_sim_) throw std::invalid_argument("Stale/future trajectory");
      trajectory_=p;yaw_=y;start_=m.start_wt_pos;yaw_start_=m.start_wt_yaw;
    } catch (const std::exception &e) {RCLCPP_ERROR(get_logger(),"Rejected trajectory: %s",e.what());}
  }
  void tick() {
    if (!requested_ || fault_) return;
    if (!odom_ok_ || px4_stamp_==0) {report("WAITING_FOR_LIO_AND_PX4");return;}
    if (elapsed(odom_rx_)>1. || elapsed(px4_rx_)>1.) {fail("Stale LIO/PX4 feedback");return;}
    if (!active_) {
      hold_.head<3>()=state_.p;ground_=state_.p.z();
      const auto r=state_.q.toRotationMatrix();hold_[3]=std::atan2(r(1,0),r(0,0));
      begin_=Steady::now();active_=true;last_step_=last_sim_;
    }
    // The landing request hands control to PX4; no OFFBOARD/rate stream is
    // sent here. Completion uses fresh FCU reports, not a drifted LIO ground z.
    if (landing_ && use_px4_landing_) {
      if (arming_==px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY && landed_ &&
          elapsed(vehicle_rx_)<1. && elapsed(land_rx_)<1.) {
        requested_=active_=false;report("LANDED");return;
      }
      report("LANDING");
      if (nav_==px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND)
        px4_land_started_=true;
      if (px4_land_started_ && !landed_ &&
          nav_!=px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
        requested_=active_=false;report("STOPPED: PX4/operator left LAND");return;
      }
      if (!px4_land_started_ && elapsed(land_requested_at_)>5.) {
        fail("PX4 did not enter LAND within 5 seconds");return;
      }
      if (!px4_land_started_ && elapsed(last_command_)>1.)
        command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
      return;
    }
    if (elapsed(begin_)>20. && arming_!=px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      fail("PX4 refused arming within 20 seconds; inspect preflight/command ACK"); return;
    }
    if (entered_offboard_ && arming_==px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
        nav_!=px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      requested_=active_=false; report("STOPPED: PX4/operator left OFFBOARD"); return;
    }
    if (nav_==px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) entered_offboard_=true;
    // Only rate control is enabled. No PX4 position, velocity or attitude setpoint.
    px4_msgs::msg::OffboardControlMode mode;mode.timestamp=stamp();mode.body_rate=true;mode_->publish(mode);
    if (elapsed(begin_)>1.2 && elapsed(last_command_)>1.) {
      if (nav_!=px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
        command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,1.,6.);
      else if (arming_!=px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
        command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,1.);
    }
    const double dt=last_sim_-last_step_;
    if (dt<control_period_*.75) return; // A paused simulator never advances its reference.
    if (dt>.25) {fail("LIO simulation time gap");return;}
    last_step_=last_sim_;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "OMMPC feedback step=%.1f ms, horizon step=%.1f ms, solver=%.2f ms",
      dt*1000.,step_*1000.,mpc_.getTimingMs());
    bool reference_ok=false;
    const bool armed=arming_==px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    if (armed && nav_==px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      if (landing_) {
        hold_[2]=std::max(ground_-.1,hold_[2]-.25*dt);
        if (landed_ && state_.p.z()<ground_+.15 && elapsed(begin_)>3.) {
          command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,0.);
          requested_=active_=false;report("LANDED");return;
        }
      } else {
        hold_[2]=std::min(takeoff_height_,hold_[2]+climb_speed_*dt);
        flight_=flight_ || (std::abs(state_.p.z()-takeoff_height_)<.15 && std::abs(state_.v.z())<.3);
      }
    }
    if (trajectory_.getPieceNum()>0 && flight_ && !landing_) {
      const double t=std::max(0.,last_sim_-start_);
      if (elapsed(heartbeat_rx_)>1.) {
        if (!allow_terminal_hold_ ||
            !zav_super::terminal_stop_reached(trajectory_,t,state_.p,state_.v)) {
          fail("SUPER trajectory heartbeat expired");return;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "SUPER terminal stop verified; holding endpoint with fresh feedback while waiting for planner");
      }
      reference_ok=mpc_.setTrajectoryReference(trajectory_,t,hold_[3],yaw_,state_,start_-yaw_start_);
      report("TRACKING");
    } else {
      reference_ok=mpc_.setHoverReference(hold_);
      report(landing_?"LANDING":(flight_?"READY":"TAKEOFF"));
    }
    ommpc_core::ControlCommand output;
    if (!reference_ok || !mpc_.execMPC(state_,last_sim_,output) || !output.bodyrates.allFinite() ||
        !std::isfinite(output.collective_acceleration)) {
      if (++failures_>=3) fail("OMMPC solver failure: "+mpc_.getLastError());
      return;
    }
    failures_=0;
    px4_msgs::msg::VehicleRatesSetpoint rates;rates.timestamp=stamp();
    // FLU -> FRD. World ENU -> NED is already accounted for by body rates.
    rates.roll=output.bodyrates.x();rates.pitch=-output.bodyrates.y();rates.yaw=-output.bodyrates.z();
    rates.thrust_body={0.f,0.f,-static_cast<float>(zav_super::thrust_command(
        std::max(0.,output.collective_acceleration),mass_,thrust_k_,scale_,idle_))};
    if (landing_) {
      const bool near_ground=state_.p.z()<ground_+.06 && std::abs(state_.v.z())<.3;
      if (near_ground && touchdown_start_<0.) touchdown_start_=last_sim_;
      if (touchdown_start_>=0.) {
        const double fade=std::clamp(1.-(last_sim_-touchdown_start_),0.,1.);
        rates.thrust_body[2]*=fade;
        rates.roll=0.;rates.pitch=0.;rates.yaw=0.;
      }
    }
    rates_->publish(rates);
  }
public:
  Controller():Node("zav_ommpc_px4") {
    const auto path="/tmp/zav-offboard-controller-"+std::to_string(getuid())+".lock";
    lock_fd_=open(path.c_str(),O_CREAT|O_RDWR,0600);
    if (lock_fd_<0 || flock(lock_fd_,LOCK_EX|LOCK_NB)!=0) throw std::runtime_error("OMMPC controller already running");
    ommpc_core::ParameterSet params;
    auto par=[this](const std::string &key,double value){return declare_parameter<double>(key,value);};
    params.mpc.step_T=step_=par("step_T",.02);
    // Keep legacy profiles unchanged unless an independent feedback period is set.
    control_period_=par("control_period",step_);
    allow_terminal_hold_=declare_parameter<bool>("allow_terminal_hold",false);
    use_px4_landing_=declare_parameter<bool>("use_px4_landing",false);
    if (!std::isfinite(control_period_) || control_period_<.01 ||
        !std::isfinite(step_) || step_<=0.)
      throw std::invalid_argument("control_period must be >= 0.01 and step_T positive");
    params.mpc.Q_pos_xy=par("Q_pos_xy",500.);params.mpc.Q_pos_z=par("Q_pos_z",800.);
    params.mpc.Q_velocity=par("Q_velocity",40.);params.mpc.Q_attitude_rp=par("Q_attitude_rp",80.);
    params.mpc.Q_attitude_yaw=par("Q_attitude_yaw",80.);params.mpc.R_thrust=par("R_thrust",.5);
    params.mpc.R_pitchroll=par("R_pitchroll",4.);params.mpc.R_yaw=par("R_yaw",2.);
    params.mpc.state_cost_exponential=par("state_cost_exponential",.5);
    params.mpc.input_cost_exponential=par("input_cost_exponential",.5);
    params.mpc.max_bodyrate_xy=par("max_bodyrate_xy",2.);params.mpc.max_bodyrate_z=par("max_bodyrate_z",1.);
    params.mpc.min_thrust=par("min_thrust",1.);params.mpc.max_thrust=par("max_thrust",30.);
    params.controller.use_fix_yaw=false;
    params.controller.velocity_frame=ommpc_core::VelocityFrame::kWorld;
    params.controller.thrust_normalization.hover_percentage=.33;
    takeoff_height_=par("takeoff_height",1.5);climb_speed_=par("climb_speed",.4);
    mass_=par("mass",1.99);thrust_k_=par("rotor_constant",5.4935e-6);
    scale_=par("rotor_scale",1937.4);idle_=par("rotor_idle",155.);
    mpc_.init(params);
    auto qos=rclcpp::SensorDataQoS();
    rates_=create_publisher<px4_msgs::msg::VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint",qos);
    mode_=create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode",qos);
    commands_=create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command",qos);
    status_=create_publisher<std_msgs::msg::String>("/zav/super/controller_status",10);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/zav/super/odom",qos,
      [this](nav_msgs::msg::Odometry::SharedPtr m){odom(*m);});
    traj_sub_=create_subscription<Poly>("/planning_cmd/poly_traj",qos,[this](Poly::SharedPtr m){trajectory(*m);});
    time_sub_=create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined",qos,
      [this](px4_msgs::msg::SensorCombined::SharedPtr m){px4_stamp_=m->timestamp;px4_rx_=Steady::now();});
    vehicle_sub_=create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status",qos,
      [this](px4_msgs::msg::VehicleStatus::SharedPtr m){nav_=m->nav_state;arming_=m->arming_state;vehicle_rx_=Steady::now();});
    land_sub_=create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",qos,
      [this](px4_msgs::msg::VehicleLandDetected::SharedPtr m){landed_=m->landed;land_rx_=Steady::now();});
    safety_sub_=create_subscription<std_msgs::msg::Bool>("/zav/lio_safety/land_requested",
      rclcpp::QoS(1).reliable().transient_local(),[this](std_msgs::msg::Bool::SharedPtr m){
        safety_latched_=safety_latched_ || m->data;
        if(safety_latched_ && requested_) fail("LIO safety landing latch");
      });
    ack_sub_=create_subscription<px4_msgs::msg::VehicleCommandAck>("/fmu/out/vehicle_command_ack",qos,
      [this](px4_msgs::msg::VehicleCommandAck::SharedPtr m){
        if(m->target_system==245 && m->target_component==191 && m->result!=0)
          RCLCPP_WARN(get_logger(),"PX4 command %u rejected/result=%u",m->command,m->result);
      });
    start_service_=create_service<std_srvs::srv::Trigger>("/zav/super/start",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r){
        r->success=odom_ok_ && elapsed(odom_rx_)<.5 && !fault_ && !safety_latched_ && !requested_;
        if(r->success) {
          requested_=true; active_=false; flight_=false; landing_=false;
          entered_offboard_=false; touchdown_start_=-1.; failures_=0;
          px4_land_started_=false;
          trajectory_.clear(); yaw_.clear();
        }
        r->message=r->success?"OMMPC takeoff requested":"Need fresh LIO and idle, healthy controller";
      });
    land_service_=create_service<std_srvs::srv::Trigger>("/zav/super/land",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r){
        r->success=active_ && !fault_;
        if(r->success){
          hold_.head<3>()=state_.p;
          const auto rotation=state_.q.toRotationMatrix();
          hold_[3]=std::atan2(rotation(1,0),rotation(0,0));
          landing_=true;trajectory_.clear();
          if (use_px4_landing_) {
            px4_land_started_=false;land_requested_at_=Steady::now();
            report("LANDING");
            command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
          }
        }
      });
    const auto polling_period=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::min(.01,control_period_*.5)));
    timer_=create_wall_timer(polling_period,[this]{tick();});
    RCLCPP_INFO(get_logger(),"OMMPC ready; /zav/super/start begins body-rate OFFBOARD takeoff");
  }
  ~Controller(){if(lock_fd_>=0) close(lock_fd_);}
};
int main(int argc,char **argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Controller>());}catch(const std::exception &e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}rclcpp::shutdown();}
