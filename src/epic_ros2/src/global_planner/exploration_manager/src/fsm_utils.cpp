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
  if (external_local_planner_ && !externalInputsReady(!external_motion_started_)) return FRONTIER_PENDING;
  // if (planner_manager_->lidar_map_interface_->getDisToOcc(fd_->odom_pos_) < planner_manager_->gcopter_config_->dilateRadiusHard)
  //   return START_FAIL;
  const bool surface_handoff=external_local_planner_ && expl_manager_->frontier_manager_ptr_->surface_continuity_;
  if (!surface_handoff && planner_manager_->topo_graph_->odom_node_->neighbors_.empty() &&
      (!external_local_planner_ || expl_manager_->active_contour_approach_.empty()))
    return START_FAIL;
  if (expl_manager_->ed_->global_tour_.size() < 2) {
    if (surface_handoff && !external_motion_started_) return FRONTIER_PENDING;
    return expl_manager_->frontier_manager_ptr_->hasInspectionFrontiers() ? FRONTIER_PENDING : NO_FRONTIER;
  }

  const Eigen::Vector3f selected=expl_manager_->ed_->next_goal_node_->center_;
  auto &measured=expl_manager_->active_contour_approach_;
  const float clearance=planner_manager_->lidar_map_interface_->getDisToOcc(selected);
  auto &front=*expl_manager_->frontier_manager_ptr_;
  if (external_local_planner_ && front.surface_continuity_ && measured.empty() &&
      clearance<planner_manager_->topo_graph_->bubble_min_radius_ && clearance>=front.measuredStanceClearance()) {
    vector<Vector3f> approach;
    if (front.connectObserved(fd_->odom_pos_,selected,.02,approach) && front.certifyFlightGuide(approach))
      measured=std::move(approach);
  }
  bool direct=external_local_planner_ && !measured.empty() && (measured.back()-selected).norm()<.01f;
  // A measured route has its own collision certificate. The graph radius
  // governs graph endpoints, while a measured endpoint uses the same flight
  // clearance that certified its incoming path. Recheck it on the latest map.
  const float required=direct ? planner_manager_->topo_graph_->parallel_bubble_astar_->safe_distance_ :
      planner_manager_->topo_graph_->bubble_min_radius_;
  if (clearance<required) {
    ROS_WARN_THROTTLE(1.,"EPIC rejected selected guide on updated map (endpoint %.2fm, required %.2fm)",clearance,required);
    updateTopoAndGlobalPath();
    return FAIL;
  }
  vector<Eigen::Vector3f> path_next_goal;

  if ((expl_manager_->ed_->next_goal_node_->center_-expl_manager_->ed_->global_tour_[1]).norm()>.01f) {
    ROS_ERROR("Selected goal node disagrees with global tour; reconnect before planning");
    expl_manager_->updateGoalNode();
    return FAIL;
  }

  int res=ParallelBubbleAstar::REACH_END;
  const auto reconnect=[&]() {
    if (!surface_handoff || !front.reconnectSelectedApproach(fd_->odom_pos_,selected,path_next_goal) ||
        !expl_manager_->guideSupported(path_next_goal)) return false;
    direct=true;
    return true;
  };
  if (direct) path_next_goal=measured;
  else if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty()) res=ParallelBubbleAstar::START_FAIL;
  else res = planner_manager_->fast_searcher_->search(planner_manager_->topo_graph_->odom_node_, fd_->odom_vel_, expl_manager_->ed_->next_goal_node_,
                                                     0.2, path_next_goal);
  if (!direct && (res!=ParallelBubbleAstar::REACH_END || path_next_goal.size()<2 ||
      (path_next_goal.back()-selected).norm()>.2f) && reconnect())
    res=ParallelBubbleAstar::REACH_END;
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

  if (external_local_planner_) {
    // SUPER follows the published polyline in order. Its first leg must not
    // lead back to the stale topology departure and create an artificial
    // reversal. Reuse the native collision-checked boundary reconnect before
    // sweep extension; preserve the selected global goal and safe suffix.
    const bool attached=!direct ?
        expl_manager_->reanchorLocalPath(path_next_goal,fd_->odom_pos_,fd_->odom_vel_) :
        expl_manager_->reanchorContour(path_next_goal,fd_->odom_pos_);
    if (!attached) {
      if (!reconnect()) {
        ROS_WARN_THROTTLE(2., "No measured connection from current pose to selected observation %.2f %.2f %.2f",
            selected.x(), selected.y(), selected.z());
        return FAIL;
      }
      ROS_INFO("Reconnected selected observation from the moving handoff pose");
    }
  } else if (!fd_->static_state_) {
    double plan_finish_time_exp = (epic_ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
    if (plan_finish_time_exp > info->duration_) {
      plan_finish_time_exp = info->duration_;
    }
    Eigen::Vector3d start_exp = info->minco_traj_.getPos(plan_finish_time_exp);
    if(!expl_manager_->reanchorLocalPath(path_next_goal,start_exp.cast<float>(),
        info->minco_traj_.getVel(plan_finish_time_exp).cast<float>()))return FAIL;
  }
  if (external_local_planner_) {
    const auto astar = planner_manager_->topo_graph_->parallel_bubble_astar_;
    // Simplify only the approach to the first observation. Retain turns,
    // backtracking, and all subsequent observation vertices in the tour.
    vector<Eigen::Vector3f> approach{path_next_goal.front()};
    for (size_t first = 0; first + 1 < path_next_goal.size();) {
      size_t last = first + 1;
      for (size_t end = last + 1; end < path_next_goal.size(); ++end) {
        const auto origin = path_next_goal[first];
        const Vector3f chord = path_next_goal[end] - origin;
        const float length = chord.norm();
        if (length < .01f) break;
        const Vector3f dir = chord / length;
        float along = 0.f;
        bool aligned = true;
        for (size_t k = first + 1; k < end; ++k) {
          const Vector3f delta = path_next_goal[k] - origin;
          const float projected = delta.dot(dir);
          if (projected < along || projected > length ||
              (delta - projected * dir).norm() > .5f * astar->safe_distance_) {
            aligned = false;
            break;
          }
          along = projected;
        }
        if (!aligned) break;
        vector<Vector3f> direct{origin, path_next_goal[end]};
        if (!astar->collisionCheck_shortenPath(direct) || !expl_manager_->guideSupported(direct)) break;
        last = end;
      }
      approach.push_back(path_next_goal[last]);
      first = last;
    }
    const auto certified=[&](const vector<Vector3f> &candidate) {
      return expl_manager_->guideSupported(candidate) && front.certifyFlightGuide(candidate);
    };
    if (!(direct ? certified(approach) : expl_manager_->repairGuide(fd_->odom_pos_,selected,approach))) {
      // Shortening is optional. A chord that cuts a checked bend must not
      // discard the original connection to the selected first observation.
      if (front.surface_continuity_ && certified(path_next_goal)) {
        approach=path_next_goal;
        ROS_INFO("Kept original measured approach after chord rejection");
      } else if (!front.reconnectSelectedApproach(fd_->odom_pos_,selected,approach) || !certified(approach)) {
        ROS_WARN_THROTTLE(1.,"EPIC rejected approach to %.2f %.2f %.2f: measured %d, raw height/visibility %d",
            selected.x(),selected.y(),selected.z(),int(direct),int(expl_manager_->guideSupported(path_next_goal)));
        return FAIL;
      }
    }
    expl_manager_->frontier_manager_ptr_->shortenReturningPass(approach);
    path_next_goal.swap(approach);
    if (!expl_manager_->frontier_manager_ptr_->surface_continuity_)
      expl_manager_->extendTourPath(path_next_goal);
  }
  if (external_local_planner_ && expl_manager_->frontier_manager_ptr_->surface_continuity_) {
    // Remove small lateral excursions across leg boundaries only when the
    // whole forward chord stays in observed air with obstacle clearance.
    const auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
    vector<Vector3f> smooth{path_next_goal.front()};
    for (size_t first=0; first+1<path_next_goal.size();) {
      size_t last=first+1;
      for (size_t end=last+1; end<path_next_goal.size(); ++end) {
        const Vector3f delta=path_next_goal[end]-path_next_goal[first];
        const float length=delta.norm();
        if (length<.01f) break;
        const Vector3f direction=delta/length;
        float previous=0.f;
        bool continuous=true;
        for (size_t k=first+1;k<end;++k) {
          const Vector3f step=path_next_goal[k]-path_next_goal[first];
          const float projection=step.dot(direction);
          if (projection<previous || projection>length ||
              (step-projection*direction).norm()>.5f*astar->safe_distance_) {
            continuous=false;
            break;
          }
          previous=projection;
        }
        vector<Vector3f> chord{path_next_goal[first],path_next_goal[end]};
        if (!continuous || !expl_manager_->guideSupported(chord) ||
            !astar->collisionCheck_shortenPath(chord)) break;
        last=end;
      }
      // Preserve each crossed viewing location's timing and yaw by projecting
      // its original vertex onto the certified chord.
      const Vector3f origin=path_next_goal[first],delta=path_next_goal[last]-origin;
      for (size_t k=first+1;k<last;++k)
        smooth.push_back(origin+delta*((path_next_goal[k]-origin).dot(delta)/delta.squaredNorm()));
      smooth.push_back(path_next_goal[last]);
      first=last;
    }
    if (expl_manager_->guideSupported(smooth) && front.certifyFlightGuide(smooth)) path_next_goal.swap(smooth);
  }
  // Extend after smoothing so the selected observation stays on the guide.
  if (external_local_planner_ && expl_manager_->frontier_manager_ptr_->surface_continuity_) {
    const bool exterior=expl_manager_->selected_observation_ && expl_manager_->selected_observation_->exterior_pass_;
    if (!exterior) expl_manager_->preservePublishedCorner(path_next_goal,fd_->odom_vel_);
    const size_t first_goal=path_next_goal.size()-1;
    const auto &preview=expl_manager_->active_contour_preview_;
    if (preview.size()>1 && (path_next_goal.back()-preview.front()).norm()<.01f &&
        expl_manager_->guideSupported(preview) &&
        expl_manager_->frontier_manager_ptr_->certifyFlightGuide(preview)) {
      path_next_goal.insert(path_next_goal.end(),preview.begin()+1,preview.end());
      expl_manager_->limitFoldedPreview(path_next_goal,first_goal);
    } else if (!exterior) {
      if (expl_manager_->extendTourPath(path_next_goal,true) &&
          expl_manager_->retracesApproach(path_next_goal,first_goal))
        path_next_goal.resize(first_goal+1);
    }
    if (!exterior) expl_manager_->extendMeasuredTurn(path_next_goal,first_goal,fd_->odom_vel_);
    FastExplorationManager::limitPreviewSlope(path_next_goal,first_goal,
        expl_manager_->frontier_manager_ptr_->getCellSize());
    if (!exterior) {
      expl_manager_->finishUnflownPass(path_next_goal,first_goal,fd_->odom_vel_);
      if (expl_manager_->continueShortRightTurn(path_next_goal)) {
        const auto &tip=path_next_goal.back();
        const Eigen::Vector3f tangent=tip-path_next_goal[path_next_goal.size()-2];
        expl_manager_->active_contour_approach_=path_next_goal;
        expl_manager_->active_contour_preview_.clear();
        expl_manager_->active_sweep_entry_=expl_manager_->short_right_turn_->first;
        expl_manager_->selected_observation_.reset();
        expl_manager_->ed_->global_tour_={path_next_goal.front(),tip};
        expl_manager_->selection_reason_="Continue through the measured right turn";
        expl_manager_->ed_->global_tour_yaws_={planner_manager_->local_data_.curr_yaw_,std::atan2(tangent.y(),tangent.x())};
        expl_manager_->updateGoalNode();
        ROS_INFO("Kept the measured right turn through its entry to %.2f %.2f",tip.x(),tip.y());
      }
    }
    if (!expl_manager_->returning_ &&
        expl_manager_->frontier_manager_ptr_->revisitsLowerPass(fd_->odom_pos_,path_next_goal)) {
      // Preview is optional. Preserve an executable first observation if
      // only its look-ahead dipped back into previously surveyed air.
      const auto goal=std::find_if(path_next_goal.begin(),path_next_goal.end(),[&](const auto &q) {
        return (q-expl_manager_->ed_->global_tour_[1]).norm()<.01f;
      });
      if (goal==path_next_goal.end()) {
        ROS_WARN_THROTTLE(2., "Cannot trim descending preview: selected observation absent from approach");
        return FAIL;
      }
      path_next_goal.erase(goal+1,path_next_goal.end());
      if (front.revisitsLowerPass(fd_->odom_pos_,path_next_goal)) {
        ROS_WARN_THROTTLE(2., "Selected approach to %.2f %.2f %.2f descends into a surveyed level from %.2fm",
            selected.x(),selected.y(),selected.z(),fd_->odom_pos_.z());
        return FAIL;
      }
    }
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
        (expl_manager_->ed_->path_next_goal_.front() - fd_->odom_pos_).norm() > .01f) {
      ROS_WARN_THROTTLE(2., "Guide departure differs from measured pose after reanchoring");
      return FAIL;
    }
    bool forward_view_yaw = false;
    epic_ros::NodeHandle nh;
    nh.param("exploration/forward_viewpoint_yaw", forward_view_yaw, false);
    const auto &route = expl_manager_->ed_->path_next_goal_;
    if (expl_manager_->frontier_manager_ptr_->surface_continuity_ &&
        std::none_of(route.begin(),route.end(),[&](const auto &point) {
          return (point-expl_manager_->ed_->global_tour_[1]).norm()<.01f;
        })) {
      ROS_WARN_THROTTLE(2., "Guide misses the selected observation %.2f %.2f %.2f", selected.x(),selected.y(),selected.z());
      return FAIL;
    }
    double yaw=fd_->odom_yaw_;
    for (size_t i = 0; i < route.size(); ++i) {
      const auto &point = route[i];
      if (!point.allFinite()) return FAIL;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = guide.header;
      if (forward_view_yaw) {
        if (i+1<route.size()) {
          const Eigen::Vector2f tangent=(route[i+1]-point).head<2>();
          if (tangent.norm()>.05f) yaw=std::atan2(tangent.y(),tangent.x());
        }
        pose.pose.orientation.z = std::sin(.5*yaw);
        pose.pose.orientation.w = std::cos(.5*yaw);
      }
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      guide.poses.push_back(pose);
    }
    if (!externalInputsReady(false)) return FAIL;
    if ((expl_manager_->ed_->path_next_goal_.front()-fd_->odom_pos_).norm() > .2f) return FAIL;
    const auto &tour=expl_manager_->ed_->global_tour_;
    if (front.surface_continuity_ && !expl_manager_->returning_ && tour.size()>1 &&
        FastExplorationManager::passedDuringPlanning(tour.front(),tour[1],route,
            planner_manager_->topo_graph_->bubble_min_radius_)) {
      ROS_INFO("Skipped first observation already crossed during planning");
      // Keep SUPER on its current trajectory until the next global solve.
      return FRONTIER_PENDING;
    }
    if (front.surface_continuity_ && !expl_manager_->returning_ && tour.size()>1) {
      const Eigen::Vector2f velocity=fd_->odom_vel_.head<2>();
      const float speed=velocity.norm();
      const auto heading=speed>=.5f ? std::optional<Eigen::Vector2f>(velocity/speed) :
                                      front.recentFlightDirection(route.front());
      if (heading) {
        const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
        const float planning_travel=(route.front()-tour.front()).head<2>().dot(*heading);
        const float reaction=std::max(planning_travel,float(speed*external_replan_period_));
        const float horizon=std::max(2.f*radius,float(reaction+
            speed*speed/(2.*std::max(1.,expl_manager_->ep_->a_avg_))));
        const Eigen::Vector2f to_goal=(tour[1]-route.front()).head<2>();
        // The instantaneous velocity can be skewed by lateral tracking. A
        // proposed reversal on measured flown air need not line up exactly
        // with that tangent all the way back to the old observation.
        const bool rolling_return=FastExplorationManager::reversesInCruise(
            route,tour[1],velocity,radius,horizon);
        const bool retraces=(rolling_return || to_goal.dot(*heading)<-.8f*to_goal.norm()) &&
            std::all_of(route.begin()+1,route.end(),[&](const auto &p) {
              return (p-route.front()).norm()<radius ||
                     front.nearFlightPath(p,1.6f*radius,0.f,.35f*radius,radius);
            });
        if (rolling_return && retraces && expl_manager_->published_first_leg_) {
          ROS_INFO("Deferred rolling reversal over flown air: speed %.2f, first (%.2f, %.2f) from (%.2f, %.2f)",
                   speed,tour[1].x(),tour[1].y(),route.front().x(),route.front().y());
          return FRONTIER_PENDING;
        }
        const auto issued=expl_manager_->publishedGuide();
        auto prefix=FastExplorationManager::shortenedForwardPrefix(
            issued,route,tour[1],*heading,radius,horizon,retraces);
        // Near braking speed, small lateral corrections skew the velocity
        // frame. The already issued straight supplies its own measured axis.
        Eigen::Vector2f check_axis=*heading;
        if (prefix.empty() && !issued.empty() && !front.flownObservationStance(issued.back())) {
          const Eigen::Vector2f remaining=(issued.back()-route.front()).head<2>();
          if (remaining.norm()>3.f*radius && heading->dot(remaining)>.60f*remaining.norm()) {
            check_axis=remaining.normalized();
            prefix=FastExplorationManager::shortenedForwardPrefix(
                issued,route,tour[1],check_axis,radius,3.f*radius,false);
          }
        }
        if (!prefix.empty() && !front.certifyFlightGuide(prefix)) {
          const Vector3f start=prefix.front(),end=prefix.back();
          prefix.clear();
          // A positional correction can offset the old guide. Certify a
          // connection to the same forward point before retaining the route.
          if (!front.connectObserved(start,end,.012,prefix) ||
              !std::all_of(prefix.begin(),prefix.end(),[&](const auto &p) {
                const Eigen::Vector2f delta=(p-start).template head<2>();
                const float along=delta.dot(check_axis);
                return along>=-.25f*radius && (delta-along*check_axis).norm()<=1.5f*radius &&
                       std::abs(p.z()-start.z())<=radius;
              }) || !front.certifyFlightGuide(prefix)) prefix.clear();
        }
        if (!prefix.empty()) {
          ROS_INFO("Kept certified forward prefix over replacement at (%.2f, %.2f, %.2f) from (%.2f, %.2f, %.2f)",
              tour[1].x(),tour[1].y(),tour[1].z(),route.front().x(),route.front().y(),route.front().z());
          return FRONTIER_PENDING;
        }
      }
    }
    if (front.surface_continuity_ && expl_manager_->retractsMeasuredEntry(route)) {
      ROS_INFO("Continuing certified inward preview until crossing the selected turn");
      return FRONTIER_PENDING;
    }
    planner_manager_->graph_visualizer_->vizTour(tour,VizColor::RED,"global");
    external_path_pub_.publish(guide);
    last_external_plan_=epic_ros::Time::now().toSec();
    if (expl_manager_->frontier_manager_ptr_->surface_continuity_) {
      auto &leg=expl_manager_->published_first_leg_;
      const auto &goal=expl_manager_->ed_->global_tour_[1];
      auto &committed=expl_manager_->published_first_route_;
      auto &preview=expl_manager_->published_preview_;
      const bool same_wall_goal=leg && (leg->second-goal).norm()<.01f;
      if (!same_wall_goal) expl_manager_->rememberDisplacedGoal(fd_->odom_pos_,route);
      if (!same_wall_goal || expl_manager_->returning_) expl_manager_->issued_wall_entry_.reset();
      if (!same_wall_goal || expl_manager_->returning_) expl_manager_->published_wall_observation_.reset();
      committed.clear();preview.clear();
      auto &entry=expl_manager_->published_sweep_entry_;
      entry.reset();
      if (expl_manager_->returning_) leg.reset();
      else {
        const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
        if (!leg || (leg->second-goal).norm()>radius) {
          const double brake_time=fd_->odom_vel_.norm()/std::max(1.,expl_manager_->ep_->a_avg_);
          expl_manager_->departure_deadline_=last_external_plan_+std::min(4.,2.+brake_time);
        }
        if (!leg || (leg->second-goal).norm()>.01f) leg=std::make_pair(route.front(),goal);
        const auto &sent=expl_manager_->ed_->path_next_goal_;
        const auto vertex=std::find_if(sent.begin(),sent.end(),[&](const auto &p){return (p-goal).norm()<.1f;});
        if (vertex!=sent.end()) {
          committed.assign(sent.begin(),vertex+1);
          preview.assign(vertex,sent.end());
          entry=expl_manager_->active_sweep_entry_;
          const auto &chosen=expl_manager_->selected_observation_;
          if (!expl_manager_->issued_wall_entry_ && chosen && !chosen->exterior_pass_ &&
              !chosen->contour_support_.empty()) {
            auto door=expl_manager_->entered_turn_;
            const auto crosses=[&](const auto &entry) {
              return entry && expl_manager_->selectedWallInsideTurn(committed,entry->first,entry->second,radius);
            };
            if (!crosses(door)) door=expl_manager_->short_right_turn_;
            if (!crosses(door)) door=front.recentRightTurn(route.front());
            if (crosses(door)) {
              float length=0.f;
              for (size_t i=1;i<committed.size();++i) length+=(committed[i]-committed[i-1]).norm();
              const double brake=fd_->odom_vel_.norm()/std::max(1.,expl_manager_->ep_->a_avg_);
              const double transit=length/std::max(1.,.5*expl_manager_->ep_->v_max_);
              expl_manager_->issued_wall_entry_=door;
              expl_manager_->issued_wall_entry_until_=last_external_plan_+std::min(8.,2.+brake+transit);
              if (std::getenv("EPIC_TRACE_WALL_CHOICE"))
                ROS_INFO("Measured interior wall at %.2f %.2f through entry %.2f %.2f",
                    goal.x(),goal.y(),door->first.x(),door->first.y());
            }
          }
          if (chosen && chosen->follows_local_outline_ && !chosen->contour_approach_.empty() &&
              (chosen->contour_approach_.back()-goal).norm()<.1f)
            expl_manager_->published_wall_observation_=chosen;

        }
      }
      if (!expl_manager_->selected_observation_ || !expl_manager_->selected_observation_->exterior_pass_)
        expl_manager_->rememberShortRightTurn(committed,preview);
      expl_manager_->frontier_manager_ptr_->confirmWallGuide(committed,preview,entry.has_value());
    }
    if (!external_motion_started_) {
      const double reference = external_reference_imu_sub_ ? external_reference_imu_stamp_ : epic_ros::Time::now().toSec();
      external_handoff_quiet_until_ = reference + external_handoff_quiet_duration_;
      external_motion_started_ = true;
    }
    visualizeDecision(true);
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
  external_exploration_complete_=false;
  if (external_local_planner_) external_complete_pub_.publish(std_msgs::msg::Bool{});
  cout << "Triggered!" << endl;
  total_time_ = epic_ros::Time::now().toSec();
  transitState(PLAN_TRAJ, "triggerCallback");
}

void FastExplorationFSM::CloudOdomCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_) {
  if (external_exploration_complete_ && !fd_->trigger_) return;
  if (external_local_planner_ && !externalInputsReady(false)) return;
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
    odometryCallback(*odom_);
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

bool FastExplorationFSM::externalInputsReady(bool settle) {
  if(!external_local_planner_) return true;
  refreshExternalOdom();
  if(external_clock_sub_) {
    rosgraph_msgs::msg::Clock latest;rclcpp::MessageInfo info;
    if(external_clock_sub_->take(latest,info))
      external_clock_stamp_=double(latest.clock.sec)+1.e-9*latest.clock.nanosec;
  }
  if(external_reference_imu_sub_) {
    sensor_msgs::msg::Imu latest;rclcpp::MessageInfo info;
    if(external_reference_imu_sub_->take(latest,info))
      external_reference_imu_stamp_=double(latest.header.stamp.sec)+1.e-9*latest.header.stamp.nanosec;
  }
  // /clock and body odometry can both be republished by the same delayed
  // adapter. Compare against the raw sensor epoch, not that circular clock.
  // The configured independent source is mandatory; do not fall back while
  // its first sample is missing. Only timestamps are used, never truth poses.
  const double reference=external_reference_imu_sub_ ? external_reference_imu_stamp_ :
      external_clock_stamp_>=0. ? external_clock_stamp_ : epic_ros::Time::now().toSec();
  const bool ready=external_state_received_ && external_input_freshness_.observe(reference,external_odom_stamp_,settle);
  if(!ready) ROS_WARN_THROTTLE(2.,"EPIC waiting for fresh live state: reference=%.3f odometry=%.3f age=%.3fs startup_confirmation=%d",
      reference,external_odom_stamp_,reference-external_odom_stamp_,int(settle));
  // A deliberately short first preview must not trigger the ordinary short-
  // remaining-route extension loop while the local pipeline is starting.
  // Keep receiving raw sensor/body state, but yield heavy work until that
  // first local handoff has had time to execute and settle.
  return ready && (!external_motion_started_ || reference>=external_handoff_quiet_until_);
}

void FastExplorationFSM::refreshExternalOdom() {
  if (!external_odom_sub_) return;
  nav_msgs::msg::Odometry latest;
  rclcpp::MessageInfo info;
  if (external_odom_sub_->take(latest,info)) odometryCallback(latest);
}

void FastExplorationFSM::odometryCallback(const nav_msgs::msg::Odometry &msg) {
  if (!external_local_planner_ || msg.header.frame_id != "world") return;
  const auto &p = msg.pose.pose.position;
  const auto &v = msg.twist.twist.linear;
  const auto &q = msg.pose.pose.orientation;
  Eigen::Vector3f position(p.x, p.y, p.z), velocity(v.x, v.y, v.z);
  Eigen::Quaternionf attitude(q.w, q.x, q.y, q.z);
  const double stamp = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec) * 1e-9;
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
  fd_->odom_yaw_ = float(tf2::getYaw(q));
  planner_manager_->local_data_.curr_pos_ = fd_->odom_pos_.cast<double>();
  planner_manager_->local_data_.curr_vel_ = fd_->odom_vel_.cast<double>();
  planner_manager_->local_data_.curr_yaw_ = fd_->odom_yaw_;
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call, bool red) {
  if (state_ == LAND && new_state != LAND) return;
  if (!fd_->trigger_ && new_state == CAUTION) return;
  if (external_local_planner_ && new_state == FINISH && state_ != FINISH &&
      fd_->trigger_ && expl_manager_->returning_) {
    const auto home = expl_manager_->frontier_manager_ptr_->home();
    if (home && (fd_->odom_pos_ - *home).norm() < .7f) {
      geometry_msgs::msg::PoseStamped returned;
      returned.header.stamp = epic_ros::Time::now();
      returned.header.frame_id = "world";
      returned.pose.position.x = home->x();
      returned.pose.position.y = home->y();
      returned.pose.position.z = home->z();
      returned.pose.orientation.w = 1.;
      external_exploration_complete_=true;
      external_return_pub_.publish(returned);
    }
  }
  int pre_s = int(state_);
  state_ = new_state;
  if (decision_pub_ && (new_state==WAIT_MAP ||
      (new_state!=pre_s && (new_state==FINISH || new_state==LAND || new_state==WAIT_TRIGGER)))) {
    visualizeDecision(false, new_state==WAIT_MAP ? "Waiting for the next reachable view" : nullptr);
  }
  if (!red) {
    cout << "\033[32m[" + pos_call + "]\033[0m: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
  } else {
    cout << "\033[31m[" + pos_call + "]\033[0m: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
  }
}

void FastExplorationFSM::visualizeDecision(bool issued, const char *status) {
  if (!decision_pub_) return;
  using Marker = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray frame;
  Marker clear;
  clear.action=Marker::DELETEALL;
  clear.header.frame_id="world";
  frame.markers.push_back(clear);
  if (!issued && !status) {
    decision_pub_->publish(frame);
    return;
  }

  const auto &manager=*expl_manager_;
  const auto &tour=manager.ed_->global_tour_;
  const auto &front=*manager.frontier_manager_ptr_;
  auto &visualizer=*planner_manager_->graph_visualizer_;
  int id=0;
  const auto point=[](const Eigen::Vector3f &p) {
    geometry_msgs::msg::Point msg;
    msg.x=p.x();msg.y=p.y();msg.z=p.z();
    return msg;
  };
  const auto label=[&](const Eigen::Vector3f &p, const std::string &text, VizColor color,
                       const char *ns, float size) {
    Marker marker;
    visualizer.SetMarker(color,ns,size,1.f,marker,1.f);
    marker.type=Marker::TEXT_VIEW_FACING;
    marker.id=id++;
    marker.pose.position=point(p);
    marker.pose.position.z+=.28;
    marker.text=text;
    frame.markers.push_back(std::move(marker));
  };
  const auto dashes=[&](const vector<Eigen::Vector3f> &path, VizColor color, const char *ns) {
    Marker marker;
    visualizer.SetMarker(color,ns,.065f,.75f,marker,1.f);
    marker.type=Marker::LINE_LIST;
    marker.id=id++;
    for (size_t i=1;i<path.size();++i) {
      const Eigen::Vector3f edge=path[i]-path[i-1];
      const int pieces=std::max(1,int(std::ceil(edge.norm()/.33f)));
      for (int j=0;j<pieces;++j) {
        marker.points.push_back(point(path[i-1]+(float(j)/pieces)*edge));
        marker.points.push_back(point(path[i-1]+((j+.6f)/pieces)*edge));
      }
    }
    if (!marker.points.empty()) frame.markers.push_back(std::move(marker));
  };
  if (issued && tour.size()>1) {
    label(tour[1],std::string("1  ")+manager.selection_reason_,VizColor::WHITE,"issued_goal",.38f);
    for (size_t i=2;i<tour.size();++i)
      label(tour[i],std::to_string(i)+"  ATSP preview",VizColor::EMERALD,"tour_order",.3f);
    for (const auto &path:manager.future_tour_paths_) {
      // Handoff may retain a prior first goal or extend a measured turn.
      // Only draw cached links that still join consecutive tour vertices.
      for (size_t i=2;i<tour.size();++i) {
        if ((path.front()-tour[i-1]).norm()<.2f && (path.back()-tour[i]).norm()<.2f) {
          dashes(path,VizColor::EMERALD,"tour_paths");
          break;
        }
      }
    }
  } else if (status) {
    label(fd_->odom_pos_,status,VizColor::WHITE,"status",.38f);
    if (manager.published_first_leg_)
      label(manager.published_first_leg_->second,"1  Last issued goal",VizColor::WHITE,"issued_goal",.38f);
  }
  for (const auto &saved:manager.deferred_observations_) {
    const auto &goal=saved->best_vp_;
    if (!goal.allFinite() || front.visitedStance(goal) || front.observedCluster(*saved)) continue;
    if (issued && tour.size()>1 && std::any_of(tour.begin()+1,tour.end(),[&](const auto &p) {
      return (p-goal).norm()<.5f;
    })) continue;
    if (!issued && manager.published_first_leg_ &&
        (manager.published_first_leg_->second-goal).norm()<.5f) continue;
    label(goal,"Saved view | recheck",VizColor::ORANGE,"saved_views",.28f);
    if (saved->contour_preview_.size()>1 && (saved->contour_preview_.front()-goal).norm()<.2f) {
      dashes(saved->contour_preview_,VizColor::ORANGE,"saved_passes");
    } else if (saved->contour_approach_.size()>1 && (saved->contour_approach_.back()-goal).norm()<.2f) {
      // Show the stored approach near the candidate without connecting the
      // current aircraft pose to a historical departure point.
      auto start=std::prev(saved->contour_approach_.end(),2);
      for (float length=(*std::next(start)-*start).norm();
           start!=saved->contour_approach_.begin() && length<2.f;) {
        --start;
        length+=(*std::next(start)-*start).norm();
      }
      dashes(vector<Eigen::Vector3f>(start,saved->contour_approach_.end()),VizColor::ORANGE,"saved_passes");
    }
  }
  decision_pub_->publish(frame);
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
