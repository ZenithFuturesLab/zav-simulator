#include <gcopter/trajectory.hpp>
#include <misc/visualizer.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <epic_ros2/msg/position_command.hpp>
#include <epic_ros2/ros.hpp>
#include <std_msgs/msg/empty.hpp>
#include <epic_ros2/msg/poly_traj.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
using namespace Eigen;
using namespace std;
double replan_time_;
double yaw_rate_limit_ = 1.2, yaw_acc_limit_ = 2.0;
epic_ros::Publisher pos_cmd_pub, cmd_vis_pub, traj_pub;
std::shared_ptr<Trajectory<7>> traj_;
std::shared_ptr<Trajectory<5>> yaw_traj_;
epic_ros2::msg::PositionCommand cmd;
std::vector<Eigen::Vector3d> traj_cmd_, traj_real_;
Eigen::Vector3d real_pos_;
double slowly_flip_yaw_target_, slowly_turn_to_center_target_;

bool receive_traj_ = false;
double traj_duration_, t_stop, yaw_traj_duration_, yaw_t_stop;
epic_ros::Time start_time_;
epic_ros::Time emergency_stop_time;
int traj_id_;
epic_ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;

double last_yaw_, last_yawdot_;

std::pair<double, double> get_yaw(double current_yaw, double t_cur, double dt) {
  std::pair<double, double> yaw_yawdot(0, 0);
  if (t_cur > yaw_traj_->getTotalDuration())
    t_cur = yaw_traj_->getTotalDuration();
  Eigen::Vector3d ap = yaw_traj_->getPos(t_cur);
  double next_yaw = ap.x();
  double d_yaw = std::remainder(next_yaw - current_yaw, 2.0 * M_PI);
  next_yaw = current_yaw + d_yaw;
  Eigen::Vector3d av = yaw_traj_->getVel(t_cur);
  const double desired = std::clamp(av(0) + 2.0 * d_yaw, -yaw_rate_limit_, yaw_rate_limit_);
  const double rate = std::clamp(desired, last_yawdot_ - yaw_acc_limit_ * dt,
                                last_yawdot_ + yaw_acc_limit_ * dt);
  yaw_yawdot.first = current_yaw + rate * dt;
  yaw_yawdot.second = rate;
  last_yawdot_ = rate;
  return yaw_yawdot;
}

void heartbeatCallback(std_msgs::msg::Empty::SharedPtr msg) {
  double fsm_time_cost = (epic_ros::Time::now() - heartbeat_time_).toSec();
  heartbeat_time_ = epic_ros::Time::now();
  static int count;
  static int last_count;
  if (count > 10000)
    count = 0;
  else
    count++;
  if (fsm_time_cost > 0.1) {
    std::cout << count - last_count << "\033[31m [FSM_Spin Time Cost] " << fsm_time_cost
              << "s \033[0m" << std::endl;
    last_count = count;
  }
}

void drawCmd(const Eigen::Vector3d &pos, const Eigen::Vector3d &vec, const int &id,
             const Eigen::Vector4d &color) {
  visualization_msgs::msg::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = epic_ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::msg::Marker::ARROW;
  mk_state.action = visualization_msgs::msg::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::msg::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

void loadPositionTrajectory(epic_ros2::msg::PolyTraj::SharedPtr msg) {
  if (msg->order != 7) {
    ROS_ERROR("[traj_server] Only support trajectory order equals 7 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size()) {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<Piece<7>::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i) {
    int i6 = i * 8;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
    msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5], msg->coef_x[i6 + 6],
    msg->coef_x[i6 + 7];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
    msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5], msg->coef_y[i6 + 6],
    msg->coef_y[i6 + 7];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
    msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5], msg->coef_z[i6 + 6],
    msg->coef_z[i6 + 7];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new Trajectory<7>(dura, cMats));

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  t_stop = traj_duration_;
  traj_id_ = msg->traj_id;

  receive_traj_ = true;
  std::vector<Eigen::Vector3d> a;
}

void loadYawTrajectory(epic_ros2::msg::PolyTraj::SharedPtr msg) {
  if (msg->order != 5) {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size()) {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }
  // std::cout << "receive yaw traj" << yaw_t_stop + start_time_.toSec() - msg->start_time.toSec()
  // << "s to go" << std::endl;

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<Piece<5>::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i) {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
    msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
    msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
    msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }
  yaw_traj_.reset(new Trajectory<5>(dura, cMats));
  yaw_traj_duration_ = yaw_traj_->getTotalDuration();
  yaw_t_stop = yaw_traj_duration_;
}

// Commit position and yaw together, including when DDS delivers topics out of order.
epic_ros2::msg::PolyTraj::SharedPtr pending_position_, pending_yaw_;
void commitPendingTrajectory() {
  if (!pending_position_ || !pending_yaw_ ||
      pending_position_->traj_id != pending_yaw_->traj_id ||
      pending_position_->start_time != pending_yaw_->start_time) return;
  if (receive_traj_ && pending_position_->traj_id <= traj_id_) return;
  if (pending_position_->order != 7 || pending_yaw_->order != 5 ||
      pending_position_->duration.empty() || pending_yaw_->duration.empty() ||
      pending_position_->coef_x.size() != pending_position_->duration.size() * 8 ||
      pending_yaw_->coef_x.size() != pending_yaw_->duration.size() * 6) return;
  const bool first = !receive_traj_;
  loadYawTrajectory(pending_yaw_);
  if (first) {
    last_yaw_ = yaw_traj_->getPos(0.0).x();
    last_yawdot_ = 0.0;
  }
  loadPositionTrajectory(pending_position_);
  pending_position_.reset();
  pending_yaw_.reset();
}
void polyTrajCallback(epic_ros2::msg::PolyTraj::SharedPtr msg) {
  if (pending_position_ && msg->traj_id < pending_position_->traj_id) return;
  pending_position_ = msg;
  commitPendingTrajectory();
}
void polyYawTrajCallback(epic_ros2::msg::PolyTraj::SharedPtr msg) {
  if (pending_yaw_ && msg->traj_id < pending_yaw_->traj_id) return;
  pending_yaw_ = msg;
  commitPendingTrajectory();
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd) {
  cmd.header.stamp = epic_ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = epic_ros2::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.jerk.x = j(0);
  cmd.jerk.y = j(1);
  cmd.jerk.z = j(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

  last_pos_ = p;
}

void cmdCallback(const epic_ros::TimerEvent &e) {
  /* no publishing before receive traj_ and have heartbeat */
  if (heartbeat_time_.toSec() <= 1e-5) {
    // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }
  if (!receive_traj_)
    return;

  epic_ros::Time time_now = epic_ros::Time::now();
  static bool printed;
  if ((time_now - heartbeat_time_).toSec() > 0.5) {
    if (!printed) {
      ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");
      printed = true;
    }
  } else {
    printed = false;
  }

  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()),
  acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static epic_ros::Time time_last = epic_ros::Time::now();

  // double traj_dur = traj_duration_;

  // if (traj_duration_ < yaw_traj_duration_)
  //   traj_dur = yaw_traj_duration_;

  // if (t_cur < traj_dur && t_cur >= 0.0) {
  if (t_cur < traj_duration_) {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);

  } else {
    pos = last_pos_;
    vel = Eigen::Vector3d::Zero();
    acc = Eigen::Vector3d::Zero();
    jer = Eigen::Vector3d::Zero();
  }

  const double dt = std::clamp((time_now - time_last).toSec(), 0.0, 0.05);
  yaw_yawdot = get_yaw(last_yaw_, std::clamp(t_cur, 0.0, yaw_traj_duration_), dt);

  time_last = time_now;
  last_yaw_ = yaw_yawdot.first;
  last_pos_ = pos;

  publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  if (traj_cmd_.size() == 0)
    traj_cmd_.emplace_back(pos);
  else if ((traj_cmd_.back() - pos).norm() > 0.02)
    traj_cmd_.emplace_back(pos);
  // if (traj_cmd_.size() > 10000)
  //   traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
  drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  // }
}

void odomCallbck(const nav_msgs::msg::Odometry &msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O")
    return;
  if (traj_real_.size() == 0) {
    traj_real_.push_back(
    Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));
  } else if ((traj_real_.back() - Eigen::Vector3d(msg.pose.pose.position.x,
                                                  msg.pose.pose.position.y,
                                                  msg.pose.pose.position.z))
             .norm() > 0.1)
    traj_real_.emplace_back(msg.pose.pose.position.x, msg.pose.pose.position.y,
                            msg.pose.pose.position.z);
  real_pos_ = traj_real_.back();

  if (traj_real_.size() > 100000)
    traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

void displayTrajWithColor(std::vector<Eigen::Vector3d> &path, double resolution,
                          Eigen::Vector4d color, int id) {
  visualization_msgs::msg::MarkerArray mk_arr;
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = epic_ros::Time::now();
  mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  if (id == 0) {
    mk.ns = "traj_exp";
    mk.action = visualization_msgs::msg::Marker::DELETEALL;
    mk.id = id;
    mk_arr.markers.emplace_back(mk);
  } else {
    mk.ns = "traj_real";
  }

  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  geometry_msgs::msg::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  mk_arr.markers.push_back(mk);
  traj_pub.publish(mk_arr);
  epic_ros::Duration(0.001).sleep();
}

void visCallback(const epic_ros::TimerEvent &e) {
  displayTrajWithColor(traj_cmd_, 0.2, Eigen::Vector4d(1, 1, 1, 1), 0);
  displayTrajWithColor(traj_real_, 0.5, Eigen::Vector4d(0, 0, 1, 1), 1);
}

void replanCallback(const std_msgs::msg::Empty &msg) {
  const double time_out = 0.1;
  epic_ros::Time time_now = epic_ros::Time::now();
  double tmp = (time_now - start_time_).toSec() + replan_time_ + time_out;
  traj_duration_ = std::min(traj_duration_, tmp);
  // yaw_traj_duration_ = std::min(yaw_traj_duration_, tmp);
  // std::cout << "\033[32m [TrajServer] Replan , stop after "
  //           << traj_duration_ - (time_now - start_time_).toSec() << "s \033[0m"
  //           << std::endl;
}

void newCallback(std_msgs::msg::Empty msg) {
  // Clear the executed traj data
  traj_cmd_.clear();
  traj_real_.clear();
}

int main(int argc, char **argv) {
  epic_ros::init(argc, argv, "traj_server");
  // epic_ros::NodeHandle node;
  epic_ros::NodeHandle nh("~");
  // epic_ros::Subscriber emergency_sub =
  //     nh.subscribe("/planning/emergency_stop", 10, emergencyStopCb);
  std::string odom_topic;
  nh.getParam("odometry_topic", odom_topic);

  epic_ros::Subscriber poly_traj_sub = nh.subscribe("/planning/trajectory", 10, polyTrajCallback);
  epic_ros::Subscriber poly_yaw_traj_sub =
  nh.subscribe("/planning/yaw_trajectory", 10, polyYawTrajCallback);
  epic_ros::Subscriber heartbeat_sub = nh.subscribe("/planning/heartbeat", 10, heartbeatCallback);
  epic_ros::Subscriber odom_sub = nh.subscribe(odom_topic, 50, odomCallbck);
  epic_ros::Subscriber replan_sub = nh.subscribe("/planning/replan", 10, replanCallback);
  epic_ros::Subscriber new_sub = nh.subscribe("planning/new", 10, newCallback);

  nh.param("/fsm/replan_time", replan_time_, 0.1);
  nh.param("yaw_max_vel", yaw_rate_limit_, 1.2);
  nh.param("yaw_max_acc", yaw_acc_limit_, 2.0);
  epic_ros::Timer vis_timer = nh.createTimer(epic_ros::Duration(0.25), visCallback);
  pos_cmd_pub = nh.advertise<epic_ros2::msg::PositionCommand>("/position_cmd", 50);
  cmd_vis_pub = nh.advertise<visualization_msgs::msg::Marker>("/planning/position_cmd_vis", 10);
  traj_pub = nh.advertise<visualization_msgs::msg::MarkerArray>("/planning/travel_traj", 10);

  epic_ros::Timer cmd_timer = nh.createTimer(epic_ros::Duration(0.01), cmdCallback);

  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;

  epic_ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  epic_ros::spin();
  pos_cmd_pub = epic_ros::Publisher();
  cmd_vis_pub = epic_ros::Publisher();
  traj_pub = epic_ros::Publisher();
  epic_ros::shutdown();

  return 0;
}