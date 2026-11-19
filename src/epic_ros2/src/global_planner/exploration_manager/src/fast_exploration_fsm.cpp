#include <epic_planner/frontier_route.h>
/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-02-29 16:54:46
 * @LastEditTime: 2024-03-11 13:22:44
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#include <epic_planner/expl_data.h>
#include <epic_planner/fast_exploration_fsm.h>
#include <epic_planner/fast_exploration_manager.h>
#include <plan_manage/planner_manager.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <traj_utils/planning_visualization.h>
using Eigen::Vector3d;
using Eigen::Vector4d;
typedef visualization_msgs::msg::Marker Marker;
typedef visualization_msgs::msg::MarkerArray MarkerArray;

void FastExplorationFSM::FSMCallback(const epic_ros::TimerEvent &e) {
  pubState();
  switch (state_) {
  case INIT: {
    if (!fd_->have_odom_) {
      ROS_WARN_THROTTLE(1.0, "no odom.");
      return;
    }
    transitState(WAIT_TRIGGER, "FSM");
    break;
  }

  case WAIT_TRIGGER: {
    ROS_WARN_THROTTLE(1.0, "wait for trigger.");
    break;
  }

  case WAIT_MAP:
  case FINISH: {
    // stopTraj();
    if (external_local_planner_) break;
    double collision_time = 0.0;
    bool safe = planner_manager_->checkTrajCollision(collision_time);
    if (!safe) {
      stopTraj();
    }
    ROS_WARN_THROTTLE(1.0, "Finished.");
    break;
  }

  case PLAN_TRAJ: {
    if (!fd_->trigger_)
      return;
    if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty()) {
      stopTraj();
      break;
    }
    if (external_local_planner_) {
      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        fd_->static_state_ = false;
        transitState(EXEC_TRAJ, "SUPER guide published");
      } else if (res == NO_FRONTIER || res == FRONTIER_PENDING) {
        transitState(res == FRONTIER_PENDING ? WAIT_MAP : FINISH, "external frontier status");
      } else {
        // SUPER owns collision checking, braking and its active trajectory.
        // A missing global guide must not invoke EPIC's unused MINCO state.
        ROS_WARN_THROTTLE(1.0, "External local planner: global guide unavailable");
      }
      break;
    }
    epic_ros::Time start = epic_ros::Time::now();
    // 要报min-step的case
    LocalTrajData *info = &planner_manager_->local_data_;
    double t_cur = (epic_ros::Time::now() - info->start_time_).toSec();
    double time_to_end = info->duration_ - t_cur;
    if (expl_manager_->ed_->global_tour_.size() == 2) {
      Eigen::Vector3f goal = expl_manager_->ed_->global_tour_[1];
      if ((goal - fd_->odom_pos_).norm() < 1e-1) {
        transitState(expl_manager_->frontier_manager_ptr_->hasInspectionFrontiers() ? WAIT_MAP : FINISH, "fsm");
        return;
      }
    }
    epic_ros::Time tplan = epic_ros::Time::now();
    exec_timer_.stop();
    int res = callExplorationPlanner();
    exec_timer_.start();
    ROS_INFO("\033[31m call planner \033[0m: %.3f",
             (epic_ros::Time::now() - tplan).toSec() * 1000.0);

    if (res == SUCCEED) {
      plan_failure_since_ = -1.0;
      poly_yaw_traj_pub_.publish(fd_->newest_yaw_traj_);
      poly_traj_pub_.publish(fd_->newest_traj_);
      fd_->static_state_ = false;
      if (fd_->use_bubble_a_star_) {
        transitState(EXEC_TRAJ,
                     "ParallelBubbleAstar plan success: new traj pub");
      } else {
        transitState(EXEC_TRAJ, "plan success: new traj pub");
      }
      fd_->use_bubble_a_star_ = false;
      fd_->half_resolution = false;

    } else if (res == NO_FRONTIER || res == FRONTIER_PENDING) {
      // if (planner_manager_->topo_graph_->global_view_points_.empty())
      transitState(res == FRONTIER_PENDING ? WAIT_MAP : FINISH, "PLAN_TRAJ: frontier status");
      const auto &active = planner_manager_->local_data_;
      fd_->static_state_ = active.minco_traj_.getPieceNum() == 0 ||
          ((epic_ros::Time::now() - active.start_time_).toSec() >= active.duration_ &&
           fd_->odom_vel_.norm() < 0.15);
    } else if (res == FAIL) {
      // A failed candidate does not invalidate a still-safe active trajectory.
      const double now = epic_ros::Time::now().toSec();
      if (plan_failure_since_ < 0.0) plan_failure_since_ = now;
      double horizon = 0.0;
      const bool safe = planner_manager_->checkTrajCollision(horizon);
      const auto &active = planner_manager_->local_data_;
      const double remaining = active.duration_ - (now - active.start_time_.toSec());
      // A fully collision-free trajectory with a stationary clear terminal
      // state continues as the trajectory server's existing endpoint hold.
      // Its end is not an impending collision. Persistent failure still lands
      // at the SAME 0.6s limit below; a real collision never uses this fallback.
      const bool safe_hold = safe && planner_manager_->hasSafeTerminalHold();
      const double safe_until = safe_hold ? std::numeric_limits<double>::infinity() :
          safe ? remaining : std::min(remaining, horizon);
      const double braking_acc = planner_manager_->gcopter_config_->maxAccMag > 0.0 ?
          planner_manager_->gcopter_config_->maxAccMag : 3.0;
      const double reaction_horizon = std::max(fp_->replan_time_ + 0.2,
          fd_->odom_vel_.norm() / braking_acc + 0.2);
      if (safe_until < reaction_horizon || now - plan_failure_since_ >= 0.6) {
        ROS_WARN("Landing: safe horizon %.3f, required %.3f, failure age %.3f",
                 safe_until, reaction_horizon, now - plan_failure_since_);
        stopTraj();
      } else {
        transitState(PLAN_TRAJ, safe_hold ? "candidate failed; continue certified stop/hold" :
                     "candidate failed; keep safe trajectory", true);
      }

    } else if (res == START_FAIL) {
      transitState(CAUTION, "PLAN_TRAJ: start failed", true);
    } else {
      cout << "330?" << endl;
    }
    break;
  }

  case EXEC_TRAJ: {
    if (external_local_planner_) break;
    // collision check
    double collision_time;
    bool safe = planner_manager_->checkTrajCollision(collision_time);
    if (!safe) {
      transitState(
          PLAN_TRAJ,
          "safetyCallback: not safe, time:" + to_string(collision_time), true);
      const double braking_acc = planner_manager_->gcopter_config_->maxAccMag > 0.0 ?
          planner_manager_->gcopter_config_->maxAccMag : 3.0;
      if (collision_time < std::max(fp_->replan_time_ + 0.2,
                                   fd_->odom_vel_.norm() / braking_acc + 0.2))
        stopTraj();
    } else if (!planner_manager_->checkTrajVelocity()) {
      transitState(PLAN_TRAJ, "velocity too fast", true);
    }

    break;
  }

  case CAUTION: {
    stopTraj();
    break;
  }
  case LAND: {
    // Non-blocking: continue heartbeat/map callbacks while PX4 owns landing.
    global_path_update_timer_.stop();
    epic_ros2::msg::TakeoffLand land_msg;
    land_msg.takeoff_land_cmd = land_msg.LAND;
    land_pub_.publish(land_msg);
    break;
  }
  }
}

void FastExplorationFSM::init(epic_ros::NodeHandle &nh,
                              FastExplorationManager::Ptr &explorer) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  nh.param("fsm/external_local_planner", external_local_planner_, false);
  if (external_local_planner_)
    external_path_pub_ = nh.advertise<nav_msgs::msg::Path>("/mission/exploration_path", 1, true);
  /*  Fsm param  */
  nh.param("fsm/thresh_replan", fp_->replan_thresh_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
  nh.param("bubble_astar/resolution_astar", fp_->bubble_a_star_resolution, 0.1);
  nh.param("fsm/emergency_replan_control_error",
           fp_->emergency_replan_control_error, 0.3);
  nh.param("fsm/replan_time_after_traj_start",
           fp_->replan_time_after_traj_start_, 0.5);
  nh.param("fsm/replan_time_before_traj_end", fp_->replan_time_before_traj_end_,
           0.5);
  /* Initialize main modules */
  // expl_manager_.reset(new FastExplorationManager);
  // expl_manager_->initialize(nh);
  expl_manager_ = explorer;
  planner_manager_ = expl_manager_->planner_manager_;

  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = {"INIT",      "WAIT_TRIGGER", "PLAN_TRAJ", "CAUTION",
                     "EXEC_TRAJ", "FINISH",       "LAND", "WAIT_MAP"};
  fd_->static_state_ = true;
  fd_->trigger_ = false;
  fd_->use_bubble_a_star_ = false;
  battary_sub_ =
      nh.subscribe("/mavros/battery", 10, &FastExplorationFSM::battaryCallback,
                   this, epic_ros::TransportHints().tcpNoDelay());

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(epic_ros::Duration(0.01),
                               &FastExplorationFSM::FSMCallback, this);
  double global_period;nh.param("global_planning/replan_period",global_period,.2);
  if(!std::isfinite(global_period) || global_period<.1 || global_period>5.)
    throw std::invalid_argument("Invalid global planning period");
  global_path_update_timer_ = nh.createTimer(
      epic_ros::Duration(global_period), &FastExplorationFSM::globalPathUpdateCallback, this);
  trigger_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1,
                              &FastExplorationFSM::triggerCallback, this);
  replan_pub_ = nh.advertise<std_msgs::msg::Empty>("/planning/replan", 10);

  heartbeat_pub_ = nh.advertise<std_msgs::msg::Empty>("/planning/heartbeat", 10);
  land_request_sub_ = nh.subscribe("/planning/land_requested", 10,
      &FastExplorationFSM::landRequestedCallback, this);
  land_pub_ =
      nh.advertise<epic_ros2::msg::TakeoffLand>("/px4ctrl/takeoff_land", 10);

  poly_traj_pub_ =
      nh.advertise<epic_ros2::msg::PolyTraj>("/planning/trajectory", 10);
  poly_yaw_traj_pub_ =
      nh.advertise<epic_ros2::msg::PolyTraj>("/planning/yaw_trajectory", 10);
  time_cost_pub_ = nh.advertise<std_msgs::msg::Float32>("/time_cost", 10);
  static_pub_ = nh.advertise<std_msgs::msg::Bool>("/planning/static", 10);
  state_pub_ = nh.advertise<visualization_msgs::msg::Marker>("/planning/state", 10);

  string odom_topic, cloud_topic;
  nh.getParam("odometry_topic", odom_topic);
  nh.getParam("cloud_topic", cloud_topic);
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(
      nh.native().get(), cloud_topic, rmw_qos_profile_sensor_data));
  auto odom_qos = rmw_qos_profile_sensor_data;
  // SUPER body odometry arrives at200Hz, while its registered cloud arrives
  // after scan processing. Ten entries retain only50ms of odometry and lose
  // the cloud's measurement epoch before the cloud reaches this subscriber.
  // Keep one second in external mode; the existing scan-rate input is unchanged.
  if (external_local_planner_) odom_qos.depth = 200;
  odom_sub_.reset(
      new message_filters::Subscriber<nav_msgs::msg::Odometry>(nh.native().get(), odom_topic, odom_qos));
  SyncPolicyCloudOdom sync_policy(external_local_planner_ ? 200 : 10);
  if (external_local_planner_)
    sync_policy.setMaxIntervalDuration(rclcpp::Duration(0, 100000000));
  sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
      static_cast<const SyncPolicyCloudOdom &>(sync_policy), *cloud_sub_, *odom_sub_));
  sync_cloud_odom_->registerCallback(
      boost::bind(&FastExplorationFSM::CloudOdomCallback, this, _1, _2));
  if (external_local_planner_) {
    fd_->odom_pos_.setZero();
    fd_->odom_vel_.setZero();
    fd_->odom_yaw_ = 0.f;
    external_odom_sub_ = nh.subscribe(odom_topic, 200,
        &FastExplorationFSM::odometryCallback, this);
  }
}

void FastExplorationFSM::battaryCallback(
    const sensor_msgs::msg::BatteryState::ConstSharedPtr &msg) {
  // if(msg->voltage < 21.0){
  //   transitState(LAND, "battary low");
}

void FastExplorationFSM::updateTopoAndGlobalPath() {
  if (!(state_ == WAIT_TRIGGER || state_ == PLAN_TRAJ || state_ == EXEC_TRAJ ||
        state_ == FINISH || state_ == WAIT_MAP)) {
    global_path_update_timer_.stop();
    // expl_manager_->frontier_manager_ptr_->viz_pocc();
    expl_manager_->frontier_manager_ptr_->visfrtcluster();
    global_path_update_timer_.start();
    return;
  }
  static int cnt = 0;
  cnt++;

  global_path_update_timer_.stop();
  epic_ros::Time t2 = epic_ros::Time::now();
  planner_manager_->topo_graph_->getRegionsToUpdate();
  // cout << "getRegionsToUpdate time cost:" << (epic_ros::Time::now() - t2).toSec()
  // * 1000 << "ms" << endl;
  planner_manager_->topo_graph_->updateSkeleton();

  epic_ros::Time t3 = epic_ros::Time::now();
  planner_manager_->topo_graph_->updateOdomNode(fd_->odom_pos_, fd_->odom_yaw_);
  planner_manager_->topo_graph_->updateHistoricalOdoms();

  if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty()) {
    double time;
    if (external_local_planner_) {
      global_path_update_timer_.start();
      return;
    }
    if (planner_manager_->local_data_.traj_id_ > 1) {
      bool safe = planner_manager_->checkTrajCollision(time);
      if (!safe) {
        transitState(CAUTION, "odom_node no nbrs");
      } else {
        global_path_update_timer_.start();

        return;
      }
    } else {
      transitState(CAUTION, "odom_node no nbrs");
    }
    global_path_update_timer_.start();
    return;
  }
  if(external_local_planner_ && expl_manager_->directional_frontiers_ && state_==EXEC_TRAJ) {
    auto suffix=remainingFrontierRoute(expl_manager_->ed_->path_next_goal_,fd_->odom_pos_,
        external_previous_position_,external_route_progress_);
    external_previous_position_=fd_->odom_pos_;
    double remaining=0.;for(size_t i=1;i<suffix.size();++i)remaining+=(suffix[i]-suffix[i-1]).norm();
    const double now=epic_ros::Time::now().toSec();
    if(external_progress_watch_stamp_<0. || external_route_progress_>external_progress_watch_+.3) {
      external_progress_watch_=external_route_progress_;
      external_progress_watch_stamp_=now;
    }
    const bool progressing=now-external_progress_watch_stamp_<3.;
    if(remaining>12. && progressing) {
      // Preserve an ongoing corridor pass, unless the next12m becomes unsafe.
      const auto prefix=FastPlannerManager::boundedGuidePrefix(suffix,12.);
      bool clear=prefix.size()>1;
      for(size_t i=1;clear && i<prefix.size();++i) {
        FrontierRoute segment{prefix[i-1].cast<float>(),prefix[i].cast<float>()};
        clear=planner_manager_->topo_graph_->parallel_bubble_astar_->collisionCheck_shortenPath(segment);
      }
      if(clear){global_path_update_timer_.start();return;}
    }
    if(remaining>.5 && !progressing) {
      const auto goal=expl_manager_->ed_->next_goal_node_->center_;
      expl_manager_->frontier_manager_ptr_->deferViewpoint(goal,15.);
      expl_manager_->external_reselect_from_current_=true;
      ROS_WARN("External route stalled %.2fs; defer viewpoint (%.2f, %.2f, %.2f) for 15s and reselect from current pose",
          now-external_progress_watch_stamp_,goal.x(),goal.y(),goal.z());
    }
  }
  if (!external_local_planner_ && planner_manager_->local_data_.traj_id_ > 1) {

    double curr_time =
        (epic_ros::Time::now() - planner_manager_->local_data_.start_time_).toSec();
    double time;
    bool safe = planner_manager_->checkTrajCollision(time);
    double total_time = planner_manager_->local_data_.duration_;
    double time2end = total_time - curr_time;

    if (safe && curr_time < fp_->replan_time_after_traj_start_ &&
        time2end > fp_->replan_time_before_traj_end_) {
      global_path_update_timer_.start();
      return;
    }
  }
  cout << endl << endl;
  cout << "\033[1;33m------------- <" << cnt
       << "> Plan Global Path start---------------" << "\033[0m" << endl;
  epic_ros::Time t4 = epic_ros::Time::now();
  // cout << "updateSkeleton time cost:" << (t3 - t2).toSec() * 1000 << "ms" <<
  // endl; if( (t3 - t1).toSec() * 1000 > 100){
  //   ROS_ERROR("time too long");
  //   exit(0);
  ROS_INFO("update topo skeleton cost: %fms, update odom vertex cost:%fms ",
           (t3 - t2).toSec() * 1000, (t4 - t3).toSec() * 1000);
  Eigen::Vector3d vel = fd_->odom_vel_.cast<double>();
  Eigen::Vector3d odom = fd_->odom_pos_.cast<double>();
  int res = expl_manager_->planGlobalPath(odom, vel);
  epic_ros::Time t5 = epic_ros::Time::now();

  cout << "\033[1;33m-------------Plan Global Path end-----------------"
       << "\033[0m" << endl
       << endl;

  planner_manager_->graph_visualizer_->vizBox(planner_manager_->topo_graph_);
  if(expl_manager_->ep_->view_graph_)
    planner_manager_->graph_visualizer_->vizGraph(planner_manager_->topo_graph_);
  std_msgs::msg::Float32 time_cost;
  double time_cost_now = (t5 - t2).toSec() * 1000;
  time_cost.data = time_cost_now;
  time_cost_pub_.publish(time_cost);

  cout << "total time cost: " << time_cost_now << "ms" << endl;
  if ((res == NO_FRONTIER || res == FRONTIER_PENDING) && state_ != WAIT_TRIGGER) {
    transitState(res == FRONTIER_PENDING ? WAIT_MAP : FINISH, "planGlobalPath: frontier status");
  } else if (res == SUCCEED && state_ != WAIT_TRIGGER) {
    transitState(PLAN_TRAJ, "planGlobalPath: succeed");
  }

  expl_manager_->frontier_manager_ptr_->viz_pocc();
  expl_manager_->frontier_manager_ptr_->visfrtcluster();
  static epic_ros::Time t_p = epic_ros::Time::now();
  if ((epic_ros::Time::now() - t_p).toSec() > 5.0) {
    expl_manager_->frontier_manager_ptr_->printMemoryCost();
    t_p = epic_ros::Time::now();
  }
  global_path_update_timer_.start();
  cout << "viz&&print cost:" << (epic_ros::Time::now() - t5).toSec() * 1000 << "ms"
       << endl;
}

void FastExplorationFSM::globalPathUpdateCallback(const epic_ros::TimerEvent &e) {
  if (state_ == LAND) return;
  updateTopoAndGlobalPath();
}
