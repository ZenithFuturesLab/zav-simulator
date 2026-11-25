#include <epic_planner/input_freshness.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-21 21:31:51
 * @LastEditTime: 2024-03-06 10:28:56
 * @Description:
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved.
 */

#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <epic_planner/fast_exploration_manager.h>
#include <iostream>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <pointcloud_topo/graph_visualizer.hpp>
#include <epic_ros2/msg/takeoff_land.hpp>
#include <epic_ros2/ros.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <string>
#include <thread>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, CAUTION, EXEC_TRAJ, FINISH, LAND, WAIT_MAP, RETURN_HOME };

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;
  double plan_failure_since_ = -1.0;

  /* ROS utils */
  epic_ros::NodeHandle node_;
  epic_ros::Timer exec_timer_, global_path_update_timer_;
  epic_ros::Subscriber trigger_sub_, map_update_sub_, battary_sub_, land_request_sub_;
  epic_ros::Publisher stop_pub_, new_pub_, replan_pub_, poly_traj_pub_, heartbeat_pub_, time_cost_pub_, poly_yaw_traj_pub_, static_pub_, state_pub_,
  land_pub_;
  double total_time_;
  bool external_local_planner_ = false;
  epic_ros::Publisher external_path_pub_, external_complete_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr decision_pub_;
  void visualizeDecision(bool issued);
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr st10_manual_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr external_odom_sub_;
  bool external_state_received_ = false;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr external_clock_sub_;
  double external_clock_stamp_=-1.;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr external_reference_imu_sub_;
  double external_reference_imu_stamp_=-1.;
  InputFreshness external_input_freshness_;
  double external_replan_period_ = 0.;
  double last_external_plan_ = -1.;
  bool external_motion_started_=false;
  bool external_exploration_complete_=false;
  rclcpp::CallbackGroup::SharedPtr external_deadline_group_;
  rclcpp::TimerBase::SharedPtr external_deadline_timer_;
  std::mutex external_plan_mutex_;
  std::optional<double> external_plan_wait_since_;
  bool external_plan_expired_ = false;
  std::optional<Eigen::Vector3f> external_home_, external_return_target_;
  std::optional<double> external_return_wait_since_;
  double next_external_return_plan_ = 0.;
  double external_handoff_quiet_duration_=3.;
  double external_handoff_quiet_until_=-1.;
  bool externalInputsReady(bool settle);
  bool externalPlanTimedOut(bool begin_wait = false);
  bool expireExternalPlanWaitLocked(double now);
  void resetExternalPlanWait();
  void startExternalReturn(const char *reason);
  void updateExternalReturn();
  bool publishExternalGuide(const vector<Eigen::Vector3f> &route);
  void finishExternalExploration(const char *reason);
  void refreshExternalOdom();
  double external_odom_stamp_ = -1.0;

  /*cloud odom callback*/
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> SyncPolicyCloudOdom;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

  SynchronizerCloudOdom sync_cloud_odom_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  void CloudOdomCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_);

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call, bool red = false);
  void battaryCallback(const sensor_msgs::msg::BatteryState::ConstSharedPtr &msg);
  /* ROS functions */
  void FSMCallback(const epic_ros::TimerEvent &e);
  void safetyCallback(const epic_ros::TimerEvent &e);
  void updateTopoAndGlobalPath();
  void globalPathUpdateCallback(const epic_ros::TimerEvent &e);
  void triggerCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
  void odometryCallback(const nav_msgs::msg::Odometry &msg);
  void stopTraj();
  void landRequestedCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);

  // void goal_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
  void visualize();
  void pubState();

public:
  FastExplorationFSM(/* args */) {}

  ~FastExplorationFSM() { sync_cloud_odom_.reset(); }

  void init(epic_ros::NodeHandle &nh, FastExplorationManager::Ptr &explorer);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace fast_planner
