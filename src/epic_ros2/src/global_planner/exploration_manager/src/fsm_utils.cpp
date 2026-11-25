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
  if (external_local_planner_ &&
      (!externalInputsReady(!external_motion_started_) || externalPlanTimedOut(true))) return FRONTIER_PENDING;
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
        const Vector3f tip=path_next_goal.back();
        const Eigen::Vector3f tangent=tip-path_next_goal[path_next_goal.size()-2];
        expl_manager_->active_contour_approach_=path_next_goal;
        expl_manager_->active_contour_preview_.clear();
        auto preview=front.previewRightWall(tip,tangent.head<2>());
        if (preview.size()>1) {
          auto through=path_next_goal;
          through.insert(through.end(),preview.begin()+1,preview.end());
          if (expl_manager_->guideSupported(through) && front.certifyFlightGuide(through) &&
              !front.revisitsLowerPass(fd_->odom_pos_,through)) {
            expl_manager_->active_contour_preview_=std::move(preview);
            path_next_goal.swap(through);
            ROS_INFO("Extended the turn entry along the next measured wall");
          }
        }
        expl_manager_->active_sweep_entry_=expl_manager_->short_right_turn_->first;
        expl_manager_->selected_observation_.reset();
        expl_manager_->ed_->global_tour_={path_next_goal.front(),tip};
        expl_manager_->selection_reason_="Continue through the measured right turn";
        expl_manager_->ed_->global_tour_yaws_={planner_manager_->local_data_.curr_yaw_,std::atan2(tangent.y(),tangent.x())};
        expl_manager_->updateGoalNode();
        ROS_INFO("Kept the measured right turn through its entry to %.2f %.2f",tip.x(),tip.y());
      }
    }
    if (expl_manager_->frontier_manager_ptr_->revisitsLowerPass(fd_->odom_pos_,path_next_goal)) {
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
  if (external_local_planner_ && expl_manager_->right_hand_priority_ &&
      path_next_goal.size()>2 && expl_manager_->ed_->global_tour_.size()>1) {
    const auto &goal=expl_manager_->ed_->global_tour_[1];
    const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
    vector<Vector3f> rounded{path_next_goal.front()};
    for (size_t i=1;i+1<path_next_goal.size();++i) {
      const auto &corner=path_next_goal[i];
      const Vector3f before=corner-path_next_goal[i-1], after=path_next_goal[i+1]-corner;
      const float lead=std::min({radius,.42f*before.norm(),.42f*after.norm()});
      if (lead<.15f || (corner-goal).norm()<.1f ||
          std::abs(before.normalized().dot(after.normalized()))>.90f) {
        rounded.push_back(corner);
        continue;
      }
      const Vector3f entry=corner-lead*before.normalized(), exit=corner+lead*after.normalized();
      vector<Vector3f> arc{rounded.back(),entry};
      for (int step=1;step<=4;++step) {
        const float t=step/4.f, u=1.f-t;
        arc.push_back(u*u*entry+2.f*u*t*corner+t*t*exit);
      }
      arc.push_back(path_next_goal[i+1]);
      if (front.certifyFlightGuide(arc))
        rounded.insert(rounded.end(),arc.begin()+1,arc.end()-1);
      else rounded.push_back(corner);
    }
    rounded.push_back(path_next_goal.back());
    if (planner_manager_->lidar_map_interface_->IsPathInBox(rounded) &&
        front.certifyFlightGuide(rounded)) path_next_goal.swap(rounded);
  }
  if (path_next_goal.size() < 2) return FRONTIER_PENDING;
  if (external_local_planner_ && !planner_manager_->lidar_map_interface_->IsPathInBox(path_next_goal)) {
    // Keep the selected observation when only the optional continuation crosses a boundary.
    const auto goal = expl_manager_->ed_->global_tour_[1];
    const auto first = std::find_if(path_next_goal.begin() + 1, path_next_goal.end(),
        [&](const Vector3f &point) { return (point - goal).norm() < .01f; });
    if (first != path_next_goal.end()) path_next_goal.erase(first + 1, path_next_goal.end());
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
    if (!fd_->odom_pos_.allFinite()) return FAIL;
    if (expl_manager_->ed_->path_next_goal_.empty() ||
        (expl_manager_->ed_->path_next_goal_.front() - fd_->odom_pos_).norm() > .01f) {
      ROS_WARN_THROTTLE(2., "Guide departure differs from measured pose after reanchoring");
      return FAIL;
    }
    const auto &route = expl_manager_->ed_->path_next_goal_;
    if (expl_manager_->frontier_manager_ptr_->surface_continuity_ &&
        std::none_of(route.begin(),route.end(),[&](const auto &point) {
          return (point-expl_manager_->ed_->global_tour_[1]).norm()<.01f;
        })) {
      ROS_WARN_THROTTLE(2., "Guide misses the selected observation %.2f %.2f %.2f", selected.x(),selected.y(),selected.z());
      return FAIL;
    }
    if (!externalInputsReady(false)) return FAIL;
    if (externalPlanTimedOut()) return FRONTIER_PENDING;
    if ((expl_manager_->ed_->path_next_goal_.front()-fd_->odom_pos_).norm() > .2f) return FAIL;
    const auto &tour=expl_manager_->ed_->global_tour_;
    bool retained_guide=false;
    if (front.surface_continuity_ && tour.size()>1) {
      const Vector2f velocity=fd_->odom_vel_.head<2>();
      const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
      const float cell=front.getCellSize();
      const auto already_crossed=FastExplorationManager::passedDuringPlanning(tour.front(),tour[1],route,radius);
      const bool reverses=FastExplorationManager::reversesInCruise(route,tour[1],velocity,radius);
      const auto last_guide=expl_manager_->publishedGuide();
      auto issued=last_guide;
      const bool issued_valid=expl_manager_->reanchorContour(issued,route.front(),true) &&
          expl_manager_->guideSupported(issued) && front.certifyFlightGuide(issued);
      const bool cuts_wall=expl_manager_->retractsMeasuredEntry(route);
      const bool shortens_wall=expl_manager_->right_hand_priority_ && issued_valid &&
          expl_manager_->published_right_hand_ &&
          FastExplorationManager::shortensIssuedGuide(issued,route,std::max(radius,2.f*cell),2.f*radius);
      // A temporary disagreement between tour solves must not stop an active
      // flight. Use the live, collision-checked prefix and look for its next edge.
      const Vector3f body=fd_->odom_pos_;
      const auto reached=[&](const vector<Vector3f> &candidate) {
        return FastExplorationManager::repeatsReachedGuide(body,last_guide,candidate,cell);
      };
      const auto safe_join=[&](const vector<Vector3f> &candidate) {
        return expl_manager_->guideSupported(candidate) && front.certifyFlightGuide(candidate) &&
            !front.revisitsLowerPass(candidate.front(),candidate);
      };
      if (issued_valid && (already_crossed || reverses || cuts_wall || shortens_wall)) {
        const float join_distance=std::max(2.f*radius,3.f*cell);
        vector<Vector3f> joined;
        if (!already_crossed) {
          joined=FastExplorationManager::spliceOnwardGuide(issued,route,tour[1],join_distance,
              [&](const vector<Vector3f> &candidate) {
                return safe_join(candidate) && !expl_manager_->retractsMeasuredEntry(candidate);
              });
          if (joined.empty()) {
            auto appended=FastExplorationManager::appendOnwardGuide(issued,route,tour[1],join_distance);
            if (safe_join(appended)) joined=std::move(appended);
          }
        }
        if (joined.empty()) {
          if (expl_manager_->right_hand_priority_ && !reached(issued)) {
            const Vector2f heading=(issued.back()-issued[issued.size()-2]).head<2>();
            expl_manager_->extendRightWallGuide(issued,heading);
          }
          if (safe_join(issued)) {
            expl_manager_->ed_->path_next_goal_.swap(issued);
            retained_guide=true;
          }
        } else {
          expl_manager_->ed_->path_next_goal_.swap(joined);
          ROS_INFO("Joined the issued wall to the next observed section");
        }
      }
      // The active trajectory may finish while the global solver is working.
      // Use the latest odometry and the issued tangent, which is still useful at rest.
      if (reached(route)) {
        vector<Vector3f> onward{body};
        const double heading_yaw=FastExplorationManager::pathTangentYaw(
            last_guide,last_guide.size()-2,2.f*cell,fd_->odom_yaw_);
        const Vector2f heading(std::cos(heading_yaw),std::sin(heading_yaw));
        if (!expl_manager_->right_hand_priority_ ||
            !expl_manager_->extendRightWallGuide(onward,heading) || !safe_join(onward)) {
          ROS_INFO("Skipped reached guide at %.2f %.2f %.2f; awaiting measured continuation",
              body.x(),body.y(),body.z());
          return FRONTIER_PENDING;
        }
        const Vector3f goal=onward.back();
        expl_manager_->active_contour_approach_=onward;
        expl_manager_->active_contour_preview_.clear();
        expl_manager_->active_sweep_entry_.reset();
        expl_manager_->selected_observation_.reset();
        expl_manager_->restored_observation_.reset();
        expl_manager_->ed_->global_tour_={body,goal};
        const double goal_yaw=FastExplorationManager::pathTangentYaw(
            onward,onward.size()-2,2.f*cell,heading_yaw);
        expl_manager_->ed_->global_tour_yaws_={fd_->odom_yaw_,goal_yaw};
        expl_manager_->selection_reason_="Continue along the measured right wall";
        expl_manager_->updateGoalNode();
        expl_manager_->ed_->path_next_goal_.swap(onward);
        retained_guide=false;
        ROS_INFO("Skipped reached guide and continued along the measured right wall to %.2f %.2f %.2f",
            goal.x(),goal.y(),goal.z());
      } else if (!retained_guide && (already_crossed ||
          FastExplorationManager::reversesInCruise(route,tour[1],velocity,radius))) {
        return FRONTIER_PENDING;
      }
    }
    if (!publishExternalGuide(route)) return FRONTIER_PENDING;
    planner_manager_->graph_visualizer_->vizTour(tour,VizColor::RED,"global");
    last_external_plan_=epic_ros::Time::now().toSec();
    if (retained_guide) {
      if (expl_manager_->published_first_leg_) {
        const auto &goal=expl_manager_->published_first_leg_->second;
        const auto split=std::find_if(route.begin(),route.end(),[&](const auto &p){return (p-goal).norm()<.01f;});
        if (split!=route.end()) {
          expl_manager_->published_first_route_.assign(route.begin(),split+1);
          expl_manager_->published_preview_.assign(split,route.end());
        }
      }
      return SUCCEED;
    }
    if (front.surface_continuity_) {
      auto &leg=expl_manager_->published_first_leg_;
      const auto &goal=expl_manager_->ed_->global_tour_[1];
      auto &committed=expl_manager_->published_first_route_;
      auto &preview=expl_manager_->published_preview_;
      const bool same_wall_goal=leg && (leg->second-goal).norm()<.01f;
      if (!same_wall_goal) expl_manager_->rememberDisplacedGoal(fd_->odom_pos_,route);
      if (!same_wall_goal) expl_manager_->issued_wall_entry_.reset();
      if (!same_wall_goal) expl_manager_->published_wall_observation_.reset();
      committed.clear();preview.clear();
      auto &entry=expl_manager_->published_sweep_entry_;
      entry.reset();
      {
        const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
        const bool new_wall_goal=!leg || (leg->second-goal).norm()>radius;
        if (new_wall_goal) {
          expl_manager_->published_right_hand_=false;
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
          if (expl_manager_->right_hand_priority_ && new_wall_goal && committed.size()>1 &&
              ((chosen && (chosen->right_hand_opening_ || chosen->exterior_pass_)) ||
               front.followsRightWall(committed))) {
            float length=0.f;
            for (size_t i=1;i<committed.size();++i) length+=(committed[i]-committed[i-1]).norm();
            expl_manager_->published_right_hand_=true;
            expl_manager_->right_hand_best_distance_=(goal-route.front()).norm();
            expl_manager_->right_hand_progress_time_=last_external_plan_;
            expl_manager_->right_hand_deadline_=last_external_plan_+
                std::clamp(2.+length/std::max(1.,.5*expl_manager_->ep_->v_max_),3.,8.);
          }

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
  resetExternalPlanWait();
  if (external_local_planner_) {
    refreshExternalOdom();
    external_home_.reset();
    if (external_state_received_ && fd_->odom_pos_.allFinite()) external_home_ = fd_->odom_pos_;
    external_complete_pub_.publish(std_msgs::msg::Bool{});
  }
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

bool FastExplorationFSM::externalPlanTimedOut(bool begin_wait) {
  if (!external_local_planner_ || !fd_->trigger_ || external_exploration_complete_ ||
      state_ == LAND || state_ == RETURN_HOME) return false;
  {
    std::lock_guard<std::mutex> lock(external_plan_mutex_);
    if (!external_plan_expired_) {
      if (!begin_wait && !external_plan_wait_since_) return false;

      // Give an issued guide time to run while the vehicle is still moving.
      refreshExternalOdom();
      if (external_motion_started_ && fd_->odom_vel_.norm() >= .3f) {
        external_plan_wait_since_.reset();
        return false;
      }
      const double now = epic_ros::Time::now().toSec();
      if (!external_plan_wait_since_ || now < *external_plan_wait_since_)
        external_plan_wait_since_ = now;
      if (!expireExternalPlanWaitLocked(now)) return false;
    }
  }
  startExternalReturn("external guide timeout");
  return true;
}

bool FastExplorationFSM::expireExternalPlanWaitLocked(double now) {
  constexpr double plan_wait_limit = 5.;
  if (external_plan_expired_ || !external_plan_wait_since_ ||
      now - *external_plan_wait_since_ < plan_wait_limit) return external_plan_expired_;

  external_plan_expired_ = true;
  ROS_WARN("EPIC exploration ended after %.2fs without a new guide", now - *external_plan_wait_since_);
  return true;
}

void FastExplorationFSM::resetExternalPlanWait() {
  std::lock_guard<std::mutex> lock(external_plan_mutex_);
  external_plan_wait_since_.reset();
  external_plan_expired_ = false;
}

bool FastExplorationFSM::publishExternalGuide(const vector<Vector3f> &route) {
  if (!planner_manager_->lidar_map_interface_->IsPathInBox(route)) {
    ROS_WARN_THROTTLE(2., "Rejected EPIC guide outside exploration bounds");
    return false;
  }
  nav_msgs::msg::Path guide;
  guide.header.frame_id = "world";
  guide.header.stamp = epic_ros::Time::now();
  const float lookahead = 2.f * expl_manager_->frontier_manager_ptr_->getCellSize();
  double yaw = fd_->odom_yaw_;
  for (size_t i = 0; i < route.size(); ++i) {
    const auto &point = route[i];
    geometry_msgs::msg::PoseStamped pose;
    pose.header = guide.header;
    yaw = FastExplorationManager::pathTangentYaw(route, i, lookahead, yaw);
    pose.pose.orientation.z = std::sin(.5 * yaw);
    pose.pose.orientation.w = std::cos(.5 * yaw);
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = point.z();
    guide.poses.push_back(pose);
  }
  std::lock_guard<std::mutex> lock(external_plan_mutex_);
  // The watchdog may expire while the main thread is building a guide.
  if (state_ != RETURN_HOME && expireExternalPlanWaitLocked(epic_ros::Time::now().toSec())) return false;
  external_path_pub_.publish(guide);
  external_plan_wait_since_.reset();
  return true;
}

void FastExplorationFSM::startExternalReturn(const char *reason) {
  resetExternalPlanWait();
  external_return_target_.reset();
  external_return_wait_since_.reset();
  next_external_return_plan_ = 0.;
  planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
  if (!external_home_) {
    finishExternalExploration("launch pose unavailable; hold position");
    return;
  }
  ROS_INFO("EPIC returning to launch %.2f %.2f %.2f", external_home_->x(), external_home_->y(), external_home_->z());
  transitState(RETURN_HOME, reason);
}

void FastExplorationFSM::updateExternalReturn() {
  const double now = epic_ros::Time::now().toSec();
  if (now < next_external_return_plan_) return;
  next_external_return_plan_ = now + external_replan_period_;

  const bool ready = externalInputsReady(false);
  if (ready && fd_->odom_vel_.norm() >= .3f) {
    external_return_wait_since_.reset();
    return;
  }
  constexpr float arrival_distance = .15f;
  if (ready && (fd_->odom_pos_ - *external_home_).norm() < arrival_distance) {
    ROS_INFO("EPIC reached launch position within %.2fm", (fd_->odom_pos_ - *external_home_).norm());
    finishExternalExploration("reached launch position");
    return;
  }
  if (ready && external_return_target_ &&
      (fd_->odom_pos_ - *external_return_target_).norm() < arrival_distance) {
    external_return_target_.reset();
    external_return_wait_since_.reset();
  }
  if (!external_return_wait_since_) external_return_wait_since_ = now;
  if (now - *external_return_wait_since_ >= 5.) {
    ROS_WARN("EPIC return stalled or has no verified connection to launch; holding position");
    finishExternalExploration("return unavailable; hold position");
    return;
  }
  if (!ready || external_return_target_) return;

  const Vector3f start = fd_->odom_pos_;
  vector<Vector3f> route;
  Vector3f home = *external_home_;
  if (!expl_manager_->planReturnLeg(start, home, route) || !externalInputsReady(false) ||
      (fd_->odom_pos_ - start).norm() > arrival_distance || fd_->odom_vel_.norm() >= .3f ||
      !publishExternalGuide(route)) return;
  external_home_ = home;
  external_return_target_ = route.back();
  external_return_wait_since_ = epic_ros::Time::now().toSec();
  planner_manager_->graph_visualizer_->vizTour(route, VizColor::RED, "global");
  ROS_INFO("EPIC published return leg to %.2f %.2f %.2f (%zu vertices)",
           route.back().x(), route.back().y(), route.back().z(), route.size());
}

void FastExplorationFSM::finishExternalExploration(const char *reason) {
  external_exploration_complete_ = true;
  resetExternalPlanWait();
  std_msgs::msg::Bool done;
  done.data = true;
  external_complete_pub_.publish(done);
  planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
  transitState(FINISH, reason);
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
  int pre_s = int(state_);
  state_ = new_state;
  if (decision_pub_ && (new_state==WAIT_MAP ||
      (new_state!=pre_s && (new_state==FINISH || new_state==LAND || new_state==RETURN_HOME || new_state==WAIT_TRIGGER)))) {
    visualizeDecision(false);
  }
  if (!red) {
    cout << "\033[32m[" + pos_call + "]\033[0m: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
  } else {
    cout << "\033[31m[" + pos_call + "]\033[0m: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
  }
}

void FastExplorationFSM::visualizeDecision(bool issued) {
  if (!decision_pub_) return;
  using Marker = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray frame;
  Marker clear;
  clear.action=Marker::DELETEALL;
  clear.header.frame_id="world";
  frame.markers.push_back(clear);

  const auto &manager=*expl_manager_;
  const auto &front=*manager.frontier_manager_ptr_;
  const auto &tour=manager.ed_->global_tour_;
  auto &visualizer=*planner_manager_->graph_visualizer_;
  const auto mark=[&](const Vector3f &stance,const char *ns,int id,VizColor color,
                      float diameter,const std::string &text,float size) {
    Marker point;
    visualizer.SetMarker(color,ns,diameter,1.f,point,1.f);
    point.type=Marker::SPHERE;
    point.id=2*id;
    point.pose.position.x=stance.x();
    point.pose.position.y=stance.y();
    point.pose.position.z=stance.z();
    frame.markers.push_back(point);
    if (text.empty()) return;
    point.type=Marker::TEXT_VIEW_FACING;
    point.id+=1;
    point.pose.position.z+=.42;
    point.scale.z=size;
    point.text=text;
    frame.markers.push_back(std::move(point));
  };

  const bool waiting=!issued && state_==WAIT_MAP;
  const size_t end=issued ? std::min(tour.size(),size_t(4)) : 0;
  for (size_t i=1;i<end;++i) {
    if (!tour[i].allFinite()) continue;
    const bool restored=i==1 && manager.restored_observation_ &&
        (*manager.restored_observation_-tour[i]).norm()<.01f;
    const auto color=restored ? VizColor::ORANGE : (i==1 ? VizColor::YELLOW : VizColor::EMERALD);
    mark(tour[i],"viewpoint_order",i,color,i==1 ? .25f : .18f,
         restored ? "1 Saved" : std::to_string(i),i==1 ? .48f : .38f);
  }
  if (issued || waiting) {
    // Show the nearest actionable memories. These stances are separate from
    // the numbered route unless the planner actually selects one as #1.
    vector<Vector3f> saved;
    for (const auto &observation:manager.deferred_observations_) {
      const auto &stance=observation->best_vp_;
      if (!stance.allFinite() || front.visitedStance(stance) || front.observedCluster(*observation) ||
          front.unseenSupport(*observation).empty()) continue;
      if (end>1 && std::any_of(tour.begin()+1,tour.begin()+end,[&](const auto &p) {
            return (p-stance).norm()<.5f;
          })) continue;
      saved.push_back(stance);
    }
    std::sort(saved.begin(),saved.end(),[&](const auto &a,const auto &b) {
      return (a-fd_->odom_pos_).squaredNorm()<(b-fd_->odom_pos_).squaredNorm();
    });
    for (size_t i=0;i<std::min(saved.size(),size_t(3));++i)
      mark(saved[i],"saved_views",i,VizColor::ORANGE,.15f,"Saved",.28f);
    const auto &pending=front.pendingSurveyLocation();
    if (waiting && pending && pending->allFinite())
      mark(*pending,"pending_survey",0,VizColor::MAGNA,.20f,"Pending check",.32f);
  }
  decision_pub_->publish(frame);
}

void FastExplorationFSM::stopTraj() {
  if (state_ == LAND) return;
  if (external_local_planner_) resetExternalPlanWait();
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
