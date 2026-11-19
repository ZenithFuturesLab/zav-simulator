#include <epic_planner/frontier_route.h>
#include <epic_planner/expl_data.h>
#include <epic_planner/fast_exploration_fsm.h>

void FastExplorationFSM::pubState() {

  std_msgs::msg::Empty heartbeat_msg;
  heartbeat_pub_.publish(heartbeat_msg);
  std_msgs::msg::Bool msg;
  msg.data = fd_->static_state_;
  static_pub_.publish(msg);
  Marker state_marker;
  state_marker.type = Marker::TEXT_VIEW_FACING;
  state_marker.pose.position.x = fd_->odom_pos_.x();
  state_marker.pose.position.y = fd_->odom_pos_.y();
  state_marker.pose.position.z = fd_->odom_pos_.z();
  state_marker.pose.orientation.w = 1.0;
  state_marker.scale.x = state_marker.scale.y = state_marker.scale.z = 0.5;
  state_marker.action = Marker::ADD;
  state_marker.color.r = 1.0;
  state_marker.color.a = 1.0;
  state_marker.text = fd_->state_str_[int(state_)];
  state_marker.header.frame_id = "world";
  state_marker.header.stamp = epic_ros::Time::now();

  state_pub_.publish(state_marker);
}

int FastExplorationFSM::callExplorationPlanner() {
  // if (planner_manager_->lidar_map_interface_->getDisToOcc(fd_->odom_pos_) < planner_manager_->gcopter_config_->dilateRadiusHard)
  //   return START_FAIL;
  if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty())
    return START_FAIL;
  if (expl_manager_->ed_->global_tour_.size() < 2)
    return expl_manager_->frontierStatus();

  // debug
  if (planner_manager_->lidar_map_interface_->getDisToOcc(expl_manager_->ed_->next_goal_node_->center_) <
      planner_manager_->topo_graph_->bubble_min_radius_) { // TODO:
    cout << "410:  next goal in occ, update it" << endl;
    updateTopoAndGlobalPath();
    return FAIL;
  }
  vector<Eigen::Vector3f> path_next_goal;

  if ((expl_manager_->ed_->next_goal_node_->center_-expl_manager_->ed_->global_tour_[1]).norm()>.01f) {
    ROS_ERROR("Selected goal node disagrees with global tour; reconnect before planning");
    expl_manager_->updateGoalNode();
    return FAIL;
  }

  int res = planner_manager_->fast_searcher_->search(planner_manager_->topo_graph_->odom_node_, fd_->odom_vel_, expl_manager_->ed_->next_goal_node_,
                                                     0.2, path_next_goal);
  if (res == ParallelBubbleAstar::NO_PATH) {
    ROS_ERROR("ExplorationPlanner: No path to goal");
    return FAIL;

  } else if (res == ParallelBubbleAstar::START_FAIL) {
    ROS_ERROR("ExplorationPlanner: Start point in occ");
    return START_FAIL;
  } else if (res == ParallelBubbleAstar::END_FAIL) {
    ROS_ERROR("ExplorationPlanner: End point in occ");
    return FAIL;
  } else if (res == ParallelBubbleAstar::TIME_OUT) {
    ROS_ERROR("ExplorationPlanner: Time out");
    return FAIL;
  }

  auto info = &planner_manager_->local_data_;

  if (path_next_goal.size()<2 ||
      (path_next_goal.back()-expl_manager_->ed_->next_goal_node_->center_).norm()>.2f) {
    ROS_ERROR("Reject local guide that does not reach selected global goal");
    return FAIL;
  }

  const double view_yaw=planner_manager_->local_data_.end_yaw_;
  epic_ros::NodeHandle observation_nh;bool forward_view_yaw=false;
  observation_nh.param("exploration/forward_viewpoint_yaw",forward_view_yaw,false);
  if (external_local_planner_) {
    if(forward_view_yaw && !std::isfinite(view_yaw))return FAIL;
    // SUPER follows the published polyline in order. Its first leg must not
    // lead back to the stale topology departure and create an artificial
    // reversal. Reuse the native collision-checked boundary reconnect before
    // sweep extension; preserve the selected global goal and safe suffix.
    if (!expl_manager_->reanchorLocalPath(path_next_goal, fd_->odom_pos_, fd_->odom_vel_))
      return FAIL;
  } else if (!fd_->static_state_) {
    double plan_finish_time_exp = (epic_ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
    if (plan_finish_time_exp > info->duration_) {
      plan_finish_time_exp = info->duration_;
    }
    Eigen::Vector3d start_exp = info->minco_traj_.getPos(plan_finish_time_exp);
    if(!expl_manager_->reanchorLocalPath(path_next_goal,start_exp.cast<float>(),
        info->minco_traj_.getVel(plan_finish_time_exp).cast<float>()))return FAIL;
  }
  if(external_local_planner_) {
    expl_manager_->extendTourPath(path_next_goal);
    if(expl_manager_->directional_frontiers_)path_next_goal=simplifyFrontierRoute(path_next_goal,1.5f,
        [&](const Eigen::Vector3f&a,const Eigen::Vector3f&b) {
          FrontierRoute segment{a,b};
          return planner_manager_->topo_graph_->parallel_bubble_astar_->collisionCheck_shortenPath(segment);
        });
  }
  vector<Eigen::Vector3f> path_next_goal_tmp;
  path_next_goal_tmp.push_back(path_next_goal[0]);

  for (int i = 1; i < path_next_goal.size();) {
    Eigen::Vector3f end_pt = path_next_goal_tmp.back();
    if ((path_next_goal[i] - end_pt).norm() > 1.0) {
      Eigen::Vector3f dir = (path_next_goal[i] - end_pt).normalized();
      path_next_goal_tmp.push_back(end_pt + 1.0 * dir);
    } else if ((path_next_goal[i] - end_pt).norm() < 0.01) {
      i++;
    } else {
      path_next_goal_tmp.push_back(path_next_goal[i]);
      i++;
    }
  }
  expl_manager_->ed_->path_next_goal_.swap(path_next_goal_tmp);
  if (external_local_planner_) {
    nav_msgs::msg::Path guide;
    guide.header.frame_id = "world";
    guide.header.stamp = epic_ros::Time::now();
    if (!fd_->odom_pos_.allFinite()) return FAIL;
    if (expl_manager_->ed_->path_next_goal_.empty() ||
        (expl_manager_->ed_->path_next_goal_.front() - fd_->odom_pos_).norm() > .01f)
      return FAIL;
    for (const auto &point : expl_manager_->ed_->path_next_goal_) {
      if (!point.allFinite()) return FAIL;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = guide.header;
      pose.pose.orientation.z = forward_view_yaw ? std::sin(.5*view_yaw) : 0.;
      pose.pose.orientation.w = forward_view_yaw ? std::cos(.5*view_yaw) : 0.;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      // Native viewpoint visibility assumes this body yaw, independently of
      // the direction of translation along the continuing inspection route.
      guide.poses.push_back(pose);
    }
    external_path_pub_.publish(guide);
    const auto target=expl_manager_->ed_->next_goal_node_->center_;
    ROS_INFO("External native goal=(%.2f, %.2f, %.2f), body=(%.2f, %.2f, %.2f), guide_vertices=%zu view_yaw=%.3f body_yaw=%.3f forward_yaw=%d",
        target.x(),target.y(),target.z(),fd_->odom_pos_.x(),fd_->odom_pos_.y(),fd_->odom_pos_.z(),guide.poses.size(),view_yaw,double(fd_->odom_yaw_),int(forward_view_yaw));
    external_route_progress_=0.;external_previous_position_=fd_->odom_pos_;
    external_progress_watch_=0.;external_progress_watch_stamp_=epic_ros::Time::now().toSec();
    return SUCCEED;
  }
  if (planner_manager_->planExploreTraj(expl_manager_->ed_->path_next_goal_, fd_->static_state_)) {
    epic_ros2::msg::PolyTraj poly_traj_msg;
    planner_manager_->polyTraj2ROSMsg(poly_traj_msg, info->start_time_);
    fd_->newest_traj_ = poly_traj_msg;
    epic_ros2::msg::PolyTraj poly_yaw_traj_msg;
    planner_manager_->polyYawTraj2ROSMsg(poly_yaw_traj_msg, info->start_time_);
    fd_->newest_yaw_traj_ = poly_yaw_traj_msg;
    return SUCCEED;
  } else {
    return FAIL;
  }
}

void FastExplorationFSM::triggerCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg) {
  if (msg->poses.empty() || msg->poses[0].pose.position.z < -0.1)
    return;

  if (state_ != WAIT_TRIGGER)
    return;
  fd_->trigger_ = true;
  expl_manager_->frontier_empty_since_=-1.;
  cout << "Triggered!" << endl;
  total_time_ = epic_ros::Time::now().toSec();
  transitState(PLAN_TRAJ, "triggerCallback");
}

void FastExplorationFSM::CloudOdomCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_) {
  epic_ros::Time t1 = epic_ros::Time::now();
  planner_manager_->lidar_map_interface_->updateCloudMapOdometry(msg, odom_);
  double collision_time;
  bool safe = external_local_planner_ || planner_manager_->checkTrajCollision(collision_time);
  if (!safe) {
    transitState(PLAN_TRAJ, "safetyCallback: not safe, time:" + to_string(collision_time), true);
    const double braking_acc = planner_manager_->gcopter_config_->maxAccMag > 0.0 ?
        planner_manager_->gcopter_config_->maxAccMag : 3.0;
    if (collision_time < std::max(fp_->replan_time_ + 0.2,
                                 fd_->odom_vel_.norm() / braking_acc + 0.2))
      stopTraj();
  }
  epic_ros::Time t2 = epic_ros::Time::now();
  epic_ros::Time t3 = epic_ros::Time::now();

  if (planner_manager_->lidar_map_interface_->ld_->lidar_cloud_.points.empty())
    return;
  auto& ld = planner_manager_->lidar_map_interface_->ld_;
  if (!external_local_planner_) {
    fd_->odom_pos_ = ld->lidar_pose_;
    fd_->odom_vel_ = ld->lidar_vel_;
    fd_->odom_yaw_ = (float)tf2::getYaw(odom_->pose.pose.orientation);
  } else if (!external_state_received_) {
    odometryCallback(odom_);
    if (!external_state_received_) return;
  }
  planner_manager_->local_data_.curr_pos_ = fd_->odom_pos_.cast<double>();
  planner_manager_->local_data_.curr_vel_ = fd_->odom_vel_.cast<double>();
  planner_manager_->topo_graph_->odom_node_->center_ = fd_->odom_pos_;
  fd_->have_odom_ = true;
  vector<ClusterInfo::Ptr> new_clusters;
  vector<int> cluster_removed;
  expl_manager_->frontier_manager_ptr_->updateFrontierClusters(new_clusters, cluster_removed);
  for (auto &cls : new_clusters) {
    cls->odom_id_ = planner_manager_->topo_graph_->history_odom_nodes_.size() - 1;
  }
  epic_ros::Time t4 = epic_ros::Time::now();

  ROS_INFO_STREAM_THROTTLE(1.0, "cloud odom callback cost: " << "ikd-tree insert:" << (t2 - t1).toSec() * 1000 << "ms  "
                                                             << "update frontier clusters: " << (t4 - t3).toSec() * 1000 << "ms  "
                                                             << "total: " << (t4 - t1).toSec() * 1000 << "ms" << endl);
}

void FastExplorationFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  if (!external_local_planner_ || msg->header.frame_id != "world") return;
  const auto &p = msg->pose.pose.position;
  const auto &v = msg->twist.twist.linear;
  const auto &q = msg->pose.pose.orientation;
  Eigen::Vector3f position(p.x, p.y, p.z), velocity(v.x, v.y, v.z);
  Eigen::Quaternionf attitude(q.w, q.x, q.y, q.z);
  const double stamp = double(msg->header.stamp.sec) + double(msg->header.stamp.nanosec) * 1e-9;
  if (!position.allFinite() || !velocity.allFinite() || !attitude.coeffs().allFinite() ||
      std::abs(attitude.norm() - 1.f) > .01f || stamp <= external_odom_stamp_) return;
  external_odom_stamp_ = stamp;
  external_state_received_ = true;
  fd_->odom_pos_ = position;
  // SUPER publishes twist in world; its matched cloud still uses its original
  // measurement epoch in updateCloudMapOdometry. Never overwrite current body
  // state with that older scan pair when selecting/publishing global guides.
  fd_->odom_vel_ = planner_manager_->lidar_map_interface_->lp_->odom_twist_in_world_ ?
      velocity : attitude * velocity;
  fd_->odom_yaw_ = float(tf2::getYaw(msg->pose.pose.orientation));
  planner_manager_->local_data_.curr_pos_ = fd_->odom_pos_.cast<double>();
  planner_manager_->local_data_.curr_vel_ = fd_->odom_vel_.cast<double>();
  planner_manager_->local_data_.curr_yaw_ = fd_->odom_yaw_;
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call, bool red) {
  if (state_ == LAND && new_state != LAND) return;
  if (!fd_->trigger_ && new_state == CAUTION) return;
  int pre_s = int(state_);
  state_ = new_state;
  if (!red) {
    cout << "\033[32m[" + pos_call + "]\033[0m: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
  } else {
    cout << "\033[31m[" + pos_call + "]\033[0m: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
  }
}

void FastExplorationFSM::stopTraj() {
  if (state_ == LAND) return;
  // Never truncate a moving polynomial into a zero-velocity hold.
  // PX4 NAV_LAND owns braking and descent once exploration becomes unsafe.
  fd_->trigger_ = false;
  transitState(LAND, "exploration stopped; request PX4 landing", true);
  epic_ros2::msg::TakeoffLand land_msg;
  land_msg.takeoff_land_cmd = land_msg.LAND;
  land_pub_.publish(land_msg);
}

void FastExplorationFSM::landRequestedCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg) {
  if (msg->data && fd_->trigger_ && state_ != LAND) stopTraj();
}
