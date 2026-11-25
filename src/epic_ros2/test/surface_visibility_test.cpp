#include <frontier_manager/frontier_manager.h>
#include <cassert>
#include <pcl_conversions/pcl_conversions.h>

struct SurfaceVisibilityRegression {
  static void run(FrontierManager &m) {
    auto node=m.nh_.native();
    if (!node->has_parameter("FrontierManager.measured_surface_visibility"))
      node->declare_parameter<bool>("FrontierManager.measured_surface_visibility",true);
    auto mode=[&](bool enabled) {
      node->set_parameter(rclcpp::Parameter("FrontierManager.measured_surface_visibility",enabled));
    };
    PointVector targets;
    Eigen::Vector3i base;
    m.pos2idx(Eigen::Vector3f(.2f,.2f,1.f),base);
    for(int y=0;y<2;++y)for(int z=0;z<2;++z) {
      const Eigen::Vector3i cell=base+Eigen::Vector3i(0,y,z);
      PointType p;m.idx2pos(cell,p);targets.push_back(p);
      ByteArrayRaw key;m.pos2bytes(p,key);m.frtd_.label_map_[key]=FRONTIER_DIS;
      PointType front;m.idx2pos(cell+Eigen::Vector3i::UnitX(),front);
      ByteArrayRaw front_key;m.pos2bytes(front,front_key);m.frtd_.label_map_[front_key]=DENSE;
    }
    const float plane_x=targets.front().x+.5f*m.frtp_.cell_size_;
    auto add_plane=[&](float x) {
      pcl::PointCloud<PointType> cloud;
      for(float y=-1;y<2;y+=.1f)for(float z=0;z<3;z+=.1f)cloud.emplace_back(x,y,z);
      auto msg=make_shared<sensor_msgs::msg::PointCloud2>();pcl::toROSMsg(cloud,*msg);
      auto odom=make_shared<nav_msgs::msg::Odometry>();odom->pose.pose.position.x=5.2;
      odom->pose.pose.position.z=1.2;odom->pose.pose.orientation.w=1.;
      m.lidar_map_interface_->updateCloudMapOdometry(msg,odom);
    };
    // The allocation-free clearance query must return the same distance as
    // one-neighbour KNN, including its 10 m cutoff and an empty map.
    const auto check_clearances=[&] {
      const std::array<float,7> offsets{-12.f,-10.f,-.02f,0.f,.37f,10.f,12.f};
      #pragma omp parallel for num_threads(2)
      for (size_t i=0;i<offsets.size();++i) {
        const float dx=offsets[i];
        for (const float y : {-.9f,.2f,1.6f}) {
          PointType query(plane_x+dx,y,1.3f);
          PointVector nearest;
          vector<float> squared;
          m.lidar_map_interface_->KNN(query,1,nearest,squared);
          const double expected=nearest.empty() ? 10. : sqrt(squared.front());
          assert(m.lidar_map_interface_->getDisToOcc(query)==expected);
          assert(m.lidar_map_interface_->getDisToOcc(query)==expected);
        }
      }
    };
    check_clearances();
    add_plane(plane_x);
    check_clearances();
    auto cluster=make_shared<ClusterInfo>();
    m.compute_cluster_info(targets,vector<Eigen::Vector3f>(4,Eigen::Vector3f::UnitX()),cluster);
    m.graph_->odom_node_=make_shared<TopoNode>();
    m.graph_->odom_node_->center_=cluster->center_+Eigen::Vector3f(5,0,0);
    m.graph_->odom_node_->yaw_=0.;
    m.graph_->bubble_min_radius_=1.f;
    {
      const Eigen::Vector3f start(plane_x-2.5f,-.5f,1.3f);
      const Eigen::Vector2f forward=Eigen::Vector2f::UnitY();
      m.graph_->bubble_min_radius_=.5f;
      assert(!m.rightWallDistance(start,forward,1.5f));
      assert(m.rightWallDistance(start,forward,3.f));
      assert(!m.rightWallDistance(start,-forward,3.f));
      m.graph_->bubble_min_radius_=1.f;
    }
    auto score=[&](float side) {
      cluster->is_dormant_=false;cluster->vp_clusters_.clear();
      ViewpointCluster group;group.center_=cluster->center_+Eigen::Vector3f(side*4,0,0);
      group.vps_.emplace_back(group.center_.x(),group.center_.y(),group.center_.z());group.distance_=4.;
      cluster->vp_clusters_.push_back(group);m.selectBestViewpoint(cluster);
      return cluster->is_reachable_;
    };
    mode(false);assert(!score(1)); // The same wall's adjacent voxel hides its own target.
    assert(score(-1)); // Legacy sign accepts the reverse side of that surface.
    mode(true);assert(score(1));
    assert(!score(-1));
    {
      const bool saved_continuity=m.surface_continuity_;
      m.surface_continuity_=true;
      for (const auto &p:targets) {
        const Eigen::Vector3i patch=(p.getVector3fMap()/(2.f*m.frtp_.cell_size_)).array().floor().cast<int>();
        m.inspected_faces_[patch]=1;
      }
      assert(!score(1));
      ViewpointCluster sampled;
      sampled.center_=cluster->center_+4.f*Eigen::Vector3f::UnitX();
      sampled.vps_.emplace_back(sampled.center_.x(),sampled.center_.y(),sampled.center_.z());
      cluster->vp_clusters_={sampled};
      cluster->is_reachable_=true;
      cluster->last_viewpoint_attempt_=1.;
      const Eigen::Vector3f upstairs=cluster->center_+4.f*Eigen::Vector3f::UnitZ();
      assert(m.surveyedLowerSurface(*cluster,upstairs));
      assert(!m.surveyedLowerSurface(*cluster,cluster->center_));
      cluster->vp_clusters_.front().center_=cluster->center_-4.f*Eigen::Vector3f::UnitX();
      assert(!m.surveyedLowerSurface(*cluster,upstairs)); // Other side of the same wall.
      m.inspected_faces_.clear();
      assert(score(1));
      m.surface_continuity_=saved_continuity;
    }
    // A second physical plane inside the local voxel band must STILL occlude.
    add_plane(plane_x+.25f);
    check_clearances();
    assert(!score(1));
    mode(false);
    std::cout<<"PASS: adjacent self-surface voxel, front/back sign, and genuine intervening wall\n";

    {
      const Eigen::Vector3i key(1,1,1);
      m.recordOpeningAttempt(key,10.);
      m.expireOpeningAttempts(10.1);assert(m.attempted_sightlines_.count(key)==1);
      m.expireOpeningAttempts(10.+std::max(.5f,m.vpp_.unreachable_retry_interval_)+.1);
      assert(m.attempted_sightlines_.count(key)==0);
    }
    {
      const Eigen::Vector3f start(0,0,3),corner(3,0,3),right(3,-2,3),left(3,2,3);
      m.flight_history_={start-3.f*Eigen::Vector3f::UnitX(),start};
      m.confirmWallGuide({start,corner,right},{right},true);
      assert(m.issued_right_turn_ && m.issued_right_turn_->second.y()<-.99f);
      const float saved_trust=m.frtp_.good_observation_trust_length_;
      m.frtp_.good_observation_trust_length_=4.5f;
      assert(m.opposesIssuedRightTurn(corner,left));
      assert(!m.opposesIssuedRightTurn(corner,right-Eigen::Vector3f::UnitY()));
      assert(!m.opposesIssuedRightTurn(corner+3.f*Eigen::Vector3f::UnitZ(),left));
      m.frtp_.good_observation_trust_length_=saved_trust;
      m.flight_history_.push_back(corner);
      m.confirmWallGuide({corner,right},{right},false);
      assert(m.issued_right_turn_ && m.issued_right_turn_->second.y()<-.99f);
      m.confirmWallGuide({corner,left},{left},false);
      assert(!m.issued_right_turn_);
      // Recover the crossed side from actual motion even if no earlier
      // goal needed protecting. Straight flight and the left fork stay free.
      m.flight_history_={start-5.f*Eigen::Vector3f::UnitX(),start,corner};
      const auto right_crossing=m.recentRightTurn(corner-3.f*Eigen::Vector3f::UnitY());
      assert(right_crossing && right_crossing->second.y()<-.9f);
      assert(!m.recentRightTurn(corner+3.f*Eigen::Vector3f::UnitY()));
      assert(!m.recentRightTurn(corner+3.f*Eigen::Vector3f::UnitX()));
      m.flight_history_.clear();
      m.confirmWallGuide({},{});
    }
    m.graph_->parallel_bubble_astar_=make_shared<ParallelBubbleAstar>();
    m.graph_->parallel_bubble_astar_->init(m.nh_,m.lidar_map_interface_);


    {
      const auto saved_air=m.observed_air_;
      const auto saved_yaw=m.graph_->odom_node_->yaw_;
      m.observed_air_.clear();
      const Eigen::Vector3f start(5.f,0.f,1.7f);
      vector<Eigen::Vector3f> first,rotated;
      assert(!m.initialSurveyGuide(start,first));
      // Only one direction has measured air. Both initial yaws must use it.
      const float cell=m.getCellSize();
      for (float y=0.f;y<5.f;y+=.25f*cell)
        m.observed_air_.insert(((start+y*Eigen::Vector3f::UnitY())/cell).array().floor().cast<int>());
      m.graph_->odom_node_->yaw_=0.f;
      assert(m.initialSurveyGuide(start,first));
      assert(first.front()==start && first.back().y()>2.f*m.graph_->bubble_min_radius_);
      assert(m.certifyFlightGuide(first));
      m.graph_->odom_node_->yaw_=float(M_PI);
      assert(m.initialSurveyGuide(start,rotated) && first==rotated);
      m.graph_->odom_node_->yaw_=saved_yaw;
      m.observed_air_=saved_air;
    }

    {
      const auto previous_air=m.observed_air_;
      assert(m.withinLevelApproach(16.60f,17.09f,17.49f,.85f));
      assert(!m.withinLevelApproach(15.90f,17.09f,17.49f,.85f));
      assert(!m.withinLevelApproach(19.f,17.09f,17.49f,.85f));
      const auto previous_body=m.surface_body_;
      const auto previous_cruise=m.graph_->parallel_bubble_astar_->safe_distance_;
      const auto previous_aperture=m.aperture_clearance_;
      const auto previous_continuity=m.surface_continuity_;
      m.aperture_clearance_=.62;
      m.graph_->parallel_bubble_astar_->safe_distance_=.72;
      m.surface_continuity_=true;
      const float face=plane_x+.25f;
      const Eigen::Vector3f start(face+.95f,0.f,1.2f),entry(face+.66f,0.f,1.2f);
      const Eigen::Vector3f exit(face+.66f,1.f,1.2f),end(face+.95f,1.f,1.2f);
      m.surface_body_=start;
      for (float x=face+.5f;x<=face+1.f;x+=.04f)
        for (float y=-.1f;y<=1.1f;y+=.04f)
          m.observed_air_.insert((Eigen::Vector3f(x,y,1.2f)/m.frtp_.cell_size_).array().floor().cast<int>());
      const vector<Eigen::Vector3f> crossing{start,entry,exit,end};
      assert(!m.certifyFlightGuide(crossing) && m.certifyShortAperture(crossing));
      m.last_aperture_path_=crossing;
      assert(m.certifyFlightGuide(crossing));
      const vector<Eigen::Vector3f> suffix{exit,end};
      assert(!m.certifyFlightGuide(suffix));
      vector<Eigen::Vector3f> handoff;
      assert(m.reconnectSelectedApproach(exit,end,handoff) && m.certifyFlightGuide(handoff));
      assert((handoff.front()-exit).norm()<1.e-5f && (handoff.back()-end).norm()<1.e-5f);
      const float saved_update_length=m.frtp_.update_length_;
      m.frtp_.update_length_=.5f;
      assert(m.reconnectSelectedApproach(start,end,handoff) && m.certifyFlightGuide(handoff));
      m.frtp_.update_length_=saved_update_length;
      m.surface_body_=start;
      m.last_aperture_path_.clear();
      assert(!m.certifyShortAperture({start,entry,exit}));
      assert(!m.certifyShortAperture({start,entry,exit,entry,exit,entry,end}));
      m.observed_air_.clear();
      assert(!m.certifyShortAperture(crossing));
      m.observed_air_=previous_air;m.surface_body_=previous_body;
      m.aperture_clearance_=previous_aperture;m.surface_continuity_=previous_continuity;
      m.graph_->parallel_bubble_astar_->safe_distance_=previous_cruise;
    }
    {
      const auto original_history=m.flight_history_;
      const auto original_air=m.observed_air_;
      const auto original_trust=m.frtp_.good_observation_trust_length_;
      m.frtp_.good_observation_trust_length_=4.5f;
      const Eigen::Vector3f shelf_side(plane_x+.7f,.3f,1.2f);
      for (float x=-1.5f;x<1.6f;x+=.1f)
        m.observed_air_.insert(((shelf_side+Eigen::Vector3f(x,0,0))/m.frtp_.cell_size_).array().floor().cast<int>());
      m.flight_history_={shelf_side-Eigen::Vector3f::UnitY(),shelf_side+Eigen::Vector3f::UnitY(),
                         shelf_side+Eigen::Vector3f(0,14,0)};
      assert(m.flownObservationStance(shelf_side+Eigen::Vector3f(.8f,0,0)));
      assert(!m.flownObservationStance(shelf_side-Eigen::Vector3f(1.4f,0,0)));
      ClusterInfo outline;
      const auto saved_enclosure=m.perimeter_pass_;
      m.perimeter_pass_.emplace();m.perimeter_pass_->level=shelf_side.z();
      assert(!m.spentSurveyObservation(outline,shelf_side));
      m.perimeter_pass_->finished=true;
      assert(m.spentSurveyObservation(outline,shelf_side));
      assert(!m.spentSurveyObservation(outline,shelf_side-Eigen::Vector3f(1.4f,0,0)));
      assert(!m.surveyedEnclosureStance(shelf_side+2.f*Eigen::Vector3f::UnitZ()));
      // The target is near the far side of the next aisle. There is room for
      // a forward turn through its centre, followed by a diagonal to the goal.
      const Eigen::Vector3f start(20,10,2),goal(14,0,2);
      const vector<Eigen::Vector3f> retreat{start,{20,-7,2},{14,-7,2},goal};
      m.perimeter_pass_->level=start.z();
      m.flight_history_={start-12.f*Eigen::Vector3f::UnitY(),start};
      auto bend=retreat;
      assert(!m.shortenReturningPass(bend) && bend==retreat);
      const float cell=m.frtp_.cell_size_;
      for (float x=13.8f;x<=20.5f;x+=.2f*cell) {
        for (float y=-.2f;y<=12.8f;y+=.2f*cell) {
          // A measured divider separates the lanes up to the near end.
          if (y<10.4f && x>16.4f && x<19.f) continue;
          // Beside the outer shelf, only a diagonal approach is observed.
          if (x<15.3f && y>2.f+(x-14.f)*5.f) continue;
          m.observed_air_.insert((Eigen::Vector3f(x,y,2)/cell).array().floor().cast<int>());
        }
      }
      assert(m.shortenReturningPass(bend));
      assert(bend.front()==start && bend.back()==goal && bend[1].y()>start.y());
      assert(m.certifyFlightGuide(bend));
      m.perimeter_pass_=saved_enclosure;
      m.flight_history_.pop_back();
      assert(!m.flownObservationStance(shelf_side));
      m.frtp_.good_observation_trust_length_=original_trust;
      m.observed_air_=original_air;m.flight_history_=original_history;
    }
    {
      const float cell=m.frtp_.cell_size_;
      const Eigen::Vector3f stance(plane_x+2.f,.05f,1.2f);
      ClusterInfo saved;
      saved.contour_approach_={stance-Eigen::Vector3f::UnitY(),stance};
      saved.contour_support_={{ObservationSample::Kind::UnknownAir,stance}};
      assert(!m.hasRevisitableSurface(saved,stance));
      const auto old_air=m.observed_air_;
      const auto old_faces=m.inspected_faces_;
      m.inspected_faces_.clear();
      for (float y: {0.15f,.15f+cell}) {
        const Eigen::Vector3f wall(plane_x+.25f,y,1.2f);
        saved.visible_cells_.emplace_back(wall.x(),wall.y(),wall.z());
        for (float u=0.f;u<=1.f;u+=.025f) {
          const Eigen::Vector3f sample=stance+u*(wall-stance);
          const Eigen::Vector3i key=(sample/cell).array().floor().cast<int>();
          for (int dy=-1;dy<=1;++dy) for (int dz=-1;dz<=1;++dz)
            m.observed_air_.insert(key+Eigen::Vector3i(0,dy,dz));
        }
      }
      assert(m.hasRevisitableSurface(saved,stance));
      for (const auto &p:saved.visible_cells_) {
        const Eigen::Vector3i key=(p.getVector3fMap()/(2.f*cell)).array().floor().cast<int>();
        m.inspected_faces_[key]=1;
      }
      assert(!m.hasRevisitableSurface(saved,stance));
      m.inspected_faces_=old_faces;m.observed_air_=old_air;
    }
    {
      const auto saved_visited=m.visited_air_;
      const auto saved_continuity=m.surface_continuity_;
      const auto saved_clearance=m.ground_clearance_;
      m.surface_continuity_=true;m.ground_clearance_=1.7;
      const Eigen::Vector3f body(30,3,12),low(32,3,8),next(38,3,15);
      for (float x=31.f;x<=33.f;x+=.1f)
        m.visited_air_.insert((Eigen::Vector3f(x,3,8)/m.frtp_.cell_size_).array().floor().cast<int>());
      assert(m.revisitsLowerPass(body,{low}));
      assert(m.revisitsLowerPass(body,{body,{30,3,8},{35,3,8},next}));
      assert(!m.revisitsLowerPass(body,{body,{32,3,11.5f},next}));
      assert(!m.revisitsLowerPass(body,{body,{32,9,8}}));
      const Eigen::Vector3f middle(32,3,9.5f),upper(32,3,11.f);
      assert(!m.revisitsLowerPass(middle,{middle,low}));
      m.highest_flown_band_=upper.z();
      assert(m.revisitsLowerPass(middle,{middle,low}));
      assert(!m.revisitsLowerPass(low,{low,upper}));
      assert(!m.revisitsLowerPass(low,{low,low+Eigen::Vector3f::UnitX()}));
      m.highest_flown_band_=body.z();
      m.lowest_flown_band_=low.z()-1.f;
      assert(m.revisitsLowerPass(body,{body,{32,9,8}}));
      assert(!m.revisitsLowerPass(body,{body,{32,9,12}}));
      m.lowest_flown_band_.reset();
      m.highest_flown_band_.reset();
      m.ground_clearance_=saved_clearance;m.surface_continuity_=saved_continuity;m.visited_air_=saved_visited;
    }
    {
      const auto saved_continuity=m.surface_continuity_;
      const auto saved_clearance=m.ground_clearance_;
      const auto saved_trust=m.frtp_.good_observation_trust_length_;
      m.surface_continuity_=true;m.ground_clearance_=1.7;m.frtp_.good_observation_trust_length_=4.5;
      const Eigen::Vector3f lower(32,3,8),body(32,3,12);
      auto label=make_shared<ClusterInfo>();
      label->is_dormant_=false;label->is_reachable_=false;label->last_viewpoint_attempt_=1.;
      label->box_min_=lower;label->box_max_=lower+Eigen::Vector3f::Constant(.4f);
      label->center_=lower;label->normal_=Eigen::Vector3f::UnitX();
      label->cells_.emplace_back(lower.x(),lower.y(),lower.z());
      m.cluster_list_.push_back(label);m.flight_history_={lower+Eigen::Vector3f::UnitX()};
      m.branch_anchors_=m.flight_history_;
      assert(m.hasInspectionFrontiers() && m.hasPendingSurveyFrontiers(body));
      ++m.next_branch_anchor_;
      assert(!m.hasPendingSurveyFrontiers(body)); // All lower residuals have been tested and all anchors checked.
      assert(m.hasPendingSurveyFrontiers(lower)); // Same-level residuals require a new sweep from this pose.
      ++m.next_branch_anchor_;
      assert(!m.hasPendingSurveyFrontiers(lower));
      assert(m.hasInspectionFrontiers()); // A skipped surface keeps its unobserved label.
      m.untested_ascent_=true;assert(m.hasPendingSurveyFrontiers(lower));m.untested_ascent_=false;
      label->box_max_.z()=lower.z()+2.f;assert(m.hasPendingSurveyFrontiers(lower));label->box_max_=lower+Eigen::Vector3f::Constant(.4f);
      m.flight_history_.clear();assert(m.hasPendingSurveyFrontiers(body));
      m.flight_history_={lower};label->last_viewpoint_attempt_=-1.;assert(m.hasPendingSurveyFrontiers(body));
      label->last_viewpoint_attempt_=1.;label->is_reachable_=true;label->vp_clusters_.emplace_back();
      label->best_vp_=body;assert(m.hasPendingSurveyFrontiers(body));
      m.cluster_list_.pop_back();
      m.next_branch_anchor_=0;m.residual_sweep_.reset();
      assert(m.hasPendingSurveyFrontiers(body)); // Empty local labels at a distant dead end.
      m.next_branch_anchor_+=m.branch_anchors_.size();
      assert(!m.hasPendingSurveyFrontiers(body));
      m.flight_history_.clear();m.branch_anchors_.clear();
      m.next_branch_anchor_=0;m.residual_sweep_.reset();
      m.frtp_.good_observation_trust_length_=saved_trust;m.surface_continuity_=saved_continuity;
      m.ground_clearance_=saved_clearance;
    }
    {
      const auto saved_air=m.observed_air_;
      const auto saved_history=m.flight_history_;
      const Eigen::Vector3f stance(6.f,6.f,1.5f), remote(20.f,6.f,1.5f);
      m.flight_history_.clear();
      const auto add_view=[&](const Eigen::Vector3f &offset) {
        m.flight_history_.push_back(stance+offset);
        for (int i=0;i<=64;++i) {
          const Eigen::Vector3f p=stance+(i/64.f)*offset;
          m.observed_air_.insert((p/m.frtp_.cell_size_).array().floor().cast<int>());
        }
      };
      add_view({2.f,0.f,0.f});add_view({-2.f,0.f,0.f});
      assert(!m.surveyedFromAround(remote,stance));
      add_view({0.f,2.f,0.f});add_view({0.f,-2.f,0.f});
      assert(m.surveyedFromAround(remote,stance));
      assert(!m.surveyedFromAround(stance+Eigen::Vector3f::UnitX(),stance));
      const float original_trust=m.frtp_.good_observation_trust_length_;
      m.frtp_.good_observation_trust_length_=3.f;
      assert(m.hasRemoteOpeningGain(remote,stance));
      const float cell=m.frtp_.cell_size_;
      const int span=static_cast<int>(std::ceil(6.f/cell))+1;
      const Eigen::Vector3i center=(stance/cell).array().floor().cast<int>();
      for (int x=-span;x<=span;++x) for (int y=-span;y<=span;++y) for (int z=-span;z<=span;++z)
        m.observed_air_.insert(center+Eigen::Vector3i(x,y,z));
      assert(!m.hasRemoteOpeningGain(remote,stance));
      assert(m.hasRemoteOpeningGain(stance+Eigen::Vector3f::UnitX(),stance));
      m.frtp_.good_observation_trust_length_=original_trust;
      m.observed_air_.clear();
      assert(!m.surveyedFromAround(remote,stance));
      m.observed_air_=saved_air;m.flight_history_=saved_history;
    }

    {
      const auto astar=m.graph_->parallel_bubble_astar_;
      const double original=astar->safe_distance_;
      astar->safe_distance_=1.05;
      m.departure_margin_=.5;
      const float wall=plane_x+.25f;
      const Eigen::Vector3f body(wall+.72f,.2f,1.5f), exit(wall+1.7f,.2f,1.5f);
      for (float x=wall+.15f;x<exit.x()+.4f;x+=.04f)
        m.observed_air_.insert((Eigen::Vector3f(x,.2f,1.5f)/m.frtp_.cell_size_).array().floor().cast<int>());
      m.surface_body_=body;
      assert(m.certifyFlightGuide({body,exit}));
      assert(!m.certifyFlightGuide({body,Eigen::Vector3f(wall+.3f,.2f,1.5f),exit}));
      assert(!m.certifyFlightGuide({body,Eigen::Vector3f(wall+.8f,.2f,1.5f)}));
      m.surface_body_=exit;
      assert(!m.certifyFlightGuide({body,exit}));
      m.surface_body_.reset();m.departure_margin_=-1.;
      // A newly filled voxel can be next to a past flight. Even two free
      // endpoints on opposite sides of a thin wall are distinct stances.
      m.flight_history_={body,body+Eigen::Vector3f(0,.1f,0)};
      for (float x=wall-.85f;x<body.x();x+=.04f)
        m.observed_air_.insert((Eigen::Vector3f(x,.2f,1.5f)/m.frtp_.cell_size_).array().floor().cast<int>());
      assert(!m.visitedStance(exit) && m.nearFlightPath(exit,1.4f,0.f,.2f));
      assert(!m.nearFlightPath(Eigen::Vector3f(wall-.55f,.2f,1.5f),1.4f,0.f,.2f));
      m.flight_history_.clear();
      astar->safe_distance_=original;
    }
    {
      const Eigen::Vector3f normal=Eigen::Vector3f::UnitX();
      const Eigen::Vector3f target(8.06f,.06f,1.66f),lidar=target+2.f*normal;
      for (int n=0;n<8;++n) m.recordFaceHit(lidar,target);
      ++m.surface_scan_;
      m.recordFaceHit(lidar,target);
      assert(!m.observedFace(target,normal));
      for (float dy:{.22f,.43f,.65f}) m.recordFaceHit(lidar,target+Eigen::Vector3f(0,dy,0));
      assert(m.observedFace(target,normal));
      assert(!m.observedFace(target,-normal));
    }
    {
      const Eigen::Vector3f origin(16.06f,8.06f,2.46f),normal=Eigen::Vector3f::UnitX();
      const Eigen::Vector3f sensor=origin+6.f*normal;
      m.frtp_.good_observation_trust_length_=4.5f;
      m.frtp_.update_length_=15.f;
      for (int scan=0;scan<4;++scan) {
        ++m.surface_scan_;
        for (int y=0;y<4;++y) for (int z=0;z<3;++z)
          m.recordFaceHit(sensor,origin+Eigen::Vector3f(0,.2f*y,.2f*z));
        assert(m.observedFace(origin,normal)==(scan==3));
      }
      assert(!m.observedFace(origin,-normal));
      assert(!m.observedFace(origin+Eigen::Vector3f(0,0,1.6f),normal));
    }
    m.visited_air_.insert((Eigen::Vector3f(2,1,1.5)/m.frtp_.cell_size_).array().floor().cast<int>());
    assert(m.visitedStance(Eigen::Vector3f(2,1,1.5)));
    assert(!m.visitedStance(Eigen::Vector3f(2,4,1.5)));
    {
      const Eigen::Vector3f body(8.f,11.5f,1.7f);
      pcl::PointCloud<PointType> returns;
      for (float x=-2.f;x<=24.f;x+=.1f) for (const float y:{9.5f,14.5f}) {
        if (y<10.f && x>11.f && x<14.f) continue;
        for (float z=1.f;z<=2.4f;z+=.2f) {
          const Eigen::Vector3f hit(x,y,z);returns.emplace_back(x,y,z);
          m.recordMeasuredHit((hit/(.5*m.frtp_.cell_size_)).array().floor().cast<int>());
        }
      }
      auto scan=make_shared<sensor_msgs::msg::PointCloud2>();pcl::toROSMsg(returns,*scan);
      auto odom=make_shared<nav_msgs::msg::Odometry>();
      odom->pose.pose.position.x=body.x();odom->pose.pose.position.y=body.y();
      odom->pose.pose.position.z=body.z();odom->pose.pose.orientation.w=1.;
      m.lidar_map_interface_->updateCloudMapOdometry(scan,odom);
      m.graph_->odom_node_->center_=body;m.graph_->odom_node_->yaw_=0.;
      for (float x=-2.f;x<=24.f;x+=.3f) for (float y=5.f;y<=16.f;y+=.3f)
        m.observed_air_.insert((Eigen::Vector3f(x,y,body.z())/m.frtp_.cell_size_).array().floor().cast<int>());
      vector<TopoNode::Ptr> candidates;
      // The global tour measures faces independently along a sweep, even
      // where the surface normal changes at the opening.
      m.appendContourViewpoints(body,candidates,true);
      assert(!candidates.empty() && candidates.size()<=2);
      const auto &wall=*m.observations_.at(candidates.front().get());
      assert(wall.fresh_contour_length_>0.f && !m.unseenSupport(wall).empty());
      // Advance along successive targets just as an approaching vehicle does.
      // The first target can be at the lip before the next one turns inside.
      Eigen::Vector3f moving=body;
      for (int i=0;i<14 && moving.y()>=9.f;++i) {
        candidates.clear();
        m.appendContourViewpoints(moving,candidates);
        if (candidates.size()!=1)
          std::cerr<<"Missing right-hand goal at "<<moving.transpose()<<" step "<<i<<std::endl;
        assert(candidates.size()==1);
        const auto &route=m.observations_.at(candidates.front().get())->contour_approach_;
        assert(m.certifyFlightGuide(route));
        const auto &preview=m.observations_.at(candidates.front().get())->contour_preview_;
        assert(!preview.empty() && (preview.front()-route.back()).norm()<1.e-4f);
        assert(preview.size()==1 || m.certifyFlightGuide(preview));
        m.confirmWallGuide(route,preview);
        const auto section=[&](size_t first,size_t last,float tolerance) {
          const Eigen::Vector3f chord=route[last]-route[first];
          if (chord.norm()<.01f || !m.certifyFlightGuide({route[first],route[last]})) return false;
          for (size_t k=first+1;k<last;++k) {
            const Eigen::Vector3f delta=route[k]-route[first];
            const float u=std::clamp(delta.dot(chord)/chord.squaredNorm(),0.f,1.f);
            if ((delta-u*chord).norm()>tolerance+1.e-4f) return false;
          }
          return true;
        };
        const size_t end=route.size()-1;
        bool valid=section(0,end,1.5f);
        float tail=0.f;
        for (size_t bend=end;!valid && bend>1;) {
          tail+=(route[bend]-route[bend-1]).norm();
          --bend;
          if (tail>2.8f) break;
          valid=section(0,bend,1.5f) && section(bend,end,.5f);
        }
        assert(valid);
        float travel=.8f;
        for (size_t k=1;k<route.size() && travel>0.f;++k) {
          const Eigen::Vector3f delta=route[k]-moving;
          if (delta.norm()<.001f) continue;
          moving+=delta.normalized()*std::min(travel,delta.norm());
          travel-=delta.norm();
        }
        m.recordPassage(moving);
      }
      assert(moving.y()<9.f && moving.x()>11.f && moving.x()<14.f);
      m.visited_air_=m.observed_air_;candidates.clear();
      m.appendContourViewpoints(moving,candidates);
      assert(candidates.empty());
    }
    {
      const Eigen::Vector3f opening(12.5f,9.5f,1.7f);
      m.flight_history_={opening-Eigen::Vector3f(0,2,0),opening,
                         opening+Eigen::Vector3f(0,6,0),opening+Eigen::Vector3f(5,6,0)};
      assert(m.needsOppositeTransit(opening,-Eigen::Vector2f::UnitY()));
      assert(!m.needsOppositeTransit(opening,Eigen::Vector2f::UnitY()));
    }
    {
      // Returns above the collision slice still prove that a wall face was seen.
      const Eigen::Vector3f upper(-7.15f,8.04f,3.32f);
      const Eigen::Vector3f sensor=upper+Eigen::Vector3f(2.f,0.f,-1.7f);
      auto &cloud=m.lidar_map_interface_->ld_->lidar_cloud_;
      cloud.clear();
      for (float y:{0.f,.2f,.4f,.6f})
        for (int beam=0;beam<3;++beam) cloud.emplace_back(upper.x(),upper.y()+y,upper.z());
      m.lidar_map_interface_->ld_->lidar_pose_=sensor;
      m.updateSurfaceSamples();m.updateSurfaceSamples();
      assert(m.observedFace(upper,Eigen::Vector3f::UnitX()));
      assert(!m.observedFace(upper,-Eigen::Vector3f::UnitX()));
      ClusterInfo face;
      face.visible_cells_.emplace_back(upper.x(),upper.y(),upper.z());
      face.normal_=Eigen::Vector3f::UnitX();
      assert(m.unseenSupport(face).empty());
      face.normal_=-face.normal_;
      assert(m.unseenSupport(face).size()==1);
      ClusterInfo saved;
      saved.contour_approach_={sensor,upper};
      saved.contour_support_={{ObservationSample::Kind::WallFace,upper,face.normal_}};
      // A native face and a retained sweep use the same ATSP support cell,
      // including at negative coordinates. New measurements expire each side.
      assert(m.unseenSupport(saved)==m.unseenSupport(face));
      saved.contour_support_.front().toward_sensor=-face.normal_;
      assert(m.unseenSupport(saved).empty());
      const Eigen::Vector3f unknown(7.43f,-17.21f,3.1f);
      const Eigen::Vector3i air_key=(unknown/m.frtp_.cell_size_).array().floor().cast<int>();
      m.observed_air_.erase(air_key);
      saved.contour_support_={{ObservationSample::Kind::UnknownAir,unknown}};
      assert(m.unseenSupport(saved)==std::vector<Eigen::Vector3i>{air_key});
      m.observed_air_.insert(air_key);
      assert(m.unseenSupport(saved).empty());
      const Eigen::Vector3i key=(upper/(.5f*m.frtp_.cell_size_)).array().floor().cast<int>();
      const auto layer=m.measured_hits_by_layer_.find(key.z());
      assert(layer==m.measured_hits_by_layer_.end() || !layer->second.count(key));
    }
    {
      // Two raster-edge remnants out of a measured wall do not require
      // another trip to the displaced stance. A frontier-sized patch does.
      ClusterInfo saved;
      saved.displaced_issued_goal_=true;
      const Eigen::Vector3f normal=Eigen::Vector3f::UnitY();
      const float resolution=2.f*m.frtp_.cell_size_;
      const int minimum=m.frtp_.cluster_minmum_point_num_;
      m.frtp_.cluster_minmum_point_num_=4;
      for (int i=0;i<24;++i) {
        const Eigen::Vector3i key(200+2*i,200,2);
        const Eigen::Vector3f point=(key.cast<float>().array()+.5f)*resolution;
        saved.contour_support_.push_back({ObservationSample::Kind::WallFace,point,normal});
        if (i<20) m.inspected_faces_[key]=0xff;
      }
      assert(!m.observedCluster(saved));
      saved.contour_support_.resize(22);
      assert(m.observedCluster(saved));
      saved.contour_support_.back().kind=ObservationSample::Kind::UnknownAir;
      assert(!m.observedCluster(saved));
      m.frtp_.cluster_minmum_point_num_=minimum;
      for (int i=0;i<20;++i) m.inspected_faces_.erase(Eigen::Vector3i(200+2*i,200,2));
    }
    {
      m.observed_air_.clear();m.visited_air_.clear();m.attempted_sightlines_.clear();
      const float cell=m.frtp_.cell_size_;
      const Eigen::Vector3i base(28,-18,4);
      const Eigen::Vector3f body=(base.cast<float>().array()+.5f)*cell;
      for (int x=0;x<=6;++x) m.observed_air_.insert(base+Eigen::Vector3i(x,0,0));
      for (int y=0;y<=6;++y) m.observed_air_.insert(base+Eigen::Vector3i(6,y,0));
      vector<Eigen::Vector3f> path;
      // A connected trail of air alone is a volumetric exploration task.
      const auto surface_limit=m.frtp_.target_max_abs_normal_z_;
      m.frtp_.target_max_abs_normal_z_=.3f;
      assert(!m.reachableOpening(body,path));
      m.frtp_.target_max_abs_normal_z_=1.f;
      assert(m.reachableOpening(body,path));
      assert((path.front()-body).norm()<.01f && (path.back()-body).norm()>2.f*cell);
      assert(m.certifyFlightGuide(path));
      assert(m.observedBoundaryGuide(body,path) && m.certifyFlightGuide(path));
      const auto old_band=m.survey_height_;m.survey_height_=body.z();
      assert(m.pendingLevelGuide(body,path) && m.certifyFlightGuide(path));
      const double previous_clearance=m.ground_clearance_;
      m.survey_height_.reset();
      m.ground_clearance_=1.7;
      assert(m.pendingLevelGuide(body,path) && m.certifyFlightGuide(path));
      m.ground_clearance_=0.;
      assert(!m.pendingLevelGuide(body,path) && path.empty());
      m.ground_clearance_=previous_clearance;
      m.survey_height_=body.z();
      {
        const auto history=m.flight_history_;
        const Eigen::Vector3f bend=body+Eigen::Vector3f(6*cell,0,0);
        const Eigen::Vector3f tip=bend+Eigen::Vector3f(0,6*cell,0);
        const Eigen::Vector3f remote=bend+Eigen::Vector3f(0,12*cell,0);
        const Eigen::Vector3i remote_key=base+Eigen::Vector3i(6,12,0);
        m.flight_history_={bend,tip};
        m.observed_air_.insert(remote_key);
        vector<Eigen::Vector3f> prefix;
        assert(!m.connectFlightHistory(body,remote,.1,path,&prefix) && path.empty());
        assert(prefix.size()>1 && (prefix.back()-tip).norm()<.01f && m.certifyFlightGuide(prefix));
        assert(m.connectFlightHistory(body,tip,.1,path,&prefix) && prefix.empty() && m.certifyFlightGuide(path));
        m.observed_air_.erase(remote_key);
        m.flight_history_=history;
      }
      m.visited_air_=m.observed_air_;
      assert(!m.pendingLevelGuide(body,path));
      m.survey_height_=old_band;
      assert(!m.observedBoundaryGuide(body,path));
      // Seeing an isolated air cell beyond a wall cannot justify a departure.
      m.observed_air_.insert(base+Eigen::Vector3i(0,0,10));
      assert(!m.observedBoundaryGuide(body,path));
      m.visited_air_.clear();
      m.observed_air_.clear();m.observed_air_.insert(base);
      assert(!m.reachableOpening(body,path));
      // An exhausted passage can be longer than the local search horizon.
      // The remote branch is eligible only while its unflown end and the
      // complete route back through previously flown air remain accessible.
      for (int x=0;x<=45;++x) {
        const auto k=(base+Eigen::Vector3i(x,0,0)).eval();
        m.observed_air_.insert(k);m.visited_air_.insert(k);
      }
      for (int y=1;y<=9;++y) m.observed_air_.insert(base+Eigen::Vector3i(45,y,0));
      const Eigen::Vector3f anchor=body+Eigen::Vector3f(45.f*cell,0,0);
      m.branch_anchors_={anchor};
      assert(!m.reachableOpening(body,path));
      assert(m.previousOpening(body,path));
      assert((path.front()-body).norm()<.01f && path.back().y()>anchor.y()+cell);
      assert(m.certifyFlightGuide(path));
      m.visited_air_=m.observed_air_;
      assert(!m.previousOpening(body,path));
      m.frtp_.target_max_abs_normal_z_=surface_limit;
    }
    {
      // From high above the original floor, past returns can still establish
      // a survey height. A wall on one side alone must not do so.
      const Eigen::Vector3f high(32.f,-10.f,7.f);
      pcl::PointCloud<PointType> cloud;
      auto insert=[&]() {
        auto scan=make_shared<sensor_msgs::msg::PointCloud2>();pcl::toROSMsg(cloud,*scan);
        auto odom=make_shared<nav_msgs::msg::Odometry>();
        odom->pose.pose.position.x=high.x();odom->pose.pose.position.y=high.y();
        odom->pose.pose.position.z=high.z();odom->pose.pose.orientation.w=1.;
        m.lidar_map_interface_->updateCloudMapOdometry(scan,odom);
        cloud.clear();
      };
      for (float y=-1.f;y<=1.f;y+=.2f) for (float z=.1f;z<7.f;z+=.2f)
        cloud.emplace_back(high.x()+1.f,high.y()+y,z);
      insert();assert(!m.measuredSurveyHeight(high,1.7f));
      for (float y=-1.4f;y<=1.4f;y+=.2f) for (float z=.1f;z<7.f;z+=.2f)
        cloud.emplace_back(high.x()-1.f,high.y()+y,z);
      insert();assert(!m.measuredSurveyHeight(high,1.7f));
      const auto old_body=m.surface_body_;
      const auto old_height=m.survey_height_;
      const auto old_clearance=m.ground_clearance_;
      const auto old_history=m.flight_history_;
      const auto old_clusters=m.cluster_list_;
      const auto old_continuity=m.surface_continuity_;
      m.surface_continuity_=true;
      m.cluster_list_.clear();
      const Eigen::Vector3f lower(high.x(),high.y(),1.7f);
      // An old pass overlaps both the start and the destination after
      // travelling far enough that its samples are outside the recent leg.
      m.flight_history_={lower-2.f*Eigen::Vector3f::UnitX(),lower,lower+20.f*Eigen::Vector3f::UnitY(),lower};
      m.ground_clearance_=1.7;
      m.survey_height_=1.7f;
      m.surface_body_=Eigen::Vector3f(high.x()-2.f, high.y(), 1.7f);
      for (float x=high.x()-2.05f;x<high.x()+.1f;x+=.1f)
        m.observed_air_.insert((Eigen::Vector3f(x,high.y(),1.7f)/m.frtp_.cell_size_).array().floor().cast<int>());
      for (float z=1.65f;z<7.2f;z+=.1f)
        m.observed_air_.insert((Eigen::Vector3f(high.x(),high.y(),z)/m.frtp_.cell_size_).array().floor().cast<int>());
      for (float floor:{0.f,4.f}) {
        for (float x=-1.4f;x<=1.4f;x+=.2f) for (float y=-1.4f;y<=1.4f;y+=.2f)
          cloud.emplace_back(high.x()+x,high.y()+y,floor);
        insert();
        const auto target=m.measuredSurveyHeight(high,1.7f);
        assert(target && std::abs(*target-floor-1.7f)<m.frtp_.cell_size_);
        assert(m.aboveSameSurveyFloor(high)==(floor==0.f));
        const Eigen::Vector3f landing(high.x(),high.y(),5.7f);
        const vector<Eigen::Vector3f> route{*m.surface_body_,landing};
        assert(m.followsRisingFloor(route)==(floor>0.f));
        assert(m.exhaustedAboveFlownFloor(route)==(floor==0.f));
        if (floor==0.f) {
          auto label=make_shared<ClusterInfo>();
          label->is_dormant_=false;label->is_reachable_=false;label->last_viewpoint_attempt_=1.;
          label->normal_=Eigen::Vector3f::UnitX();
          label->center_=lower+2.f*Eigen::Vector3f::UnitX();
          label->box_min_=label->center_;label->box_max_=label->center_;
          label->cells_.emplace_back(label->center_.x(),label->center_.y(),label->center_.z());
          m.cluster_list_.push_back(label);
          assert(m.exhaustedAboveFlownFloor(route));
          label->last_viewpoint_attempt_=-1.;
          assert(!m.exhaustedAboveFlownFloor(route));
          label->last_viewpoint_attempt_=1.;label->is_reachable_=true;label->vp_clusters_.emplace_back();
          assert(!m.exhaustedAboveFlownFloor(route));
          label->is_reachable_=false;label->vp_clusters_.clear();label->cells_.front().x+=10.f;
          assert(!m.exhaustedAboveFlownFloor(route));
          m.cluster_list_.clear();
        }
      }
      m.flight_history_=old_history;
      m.cluster_list_=old_clusters;
      m.surface_continuity_=old_continuity;
      m.ground_clearance_=old_clearance;
      m.surface_body_=old_body;
      m.survey_height_=old_height;
    }
    {
      // A safe straight line fits between the coarse voxel centres. Refining
      // the search lattice must retain the original collision clearance.
      const Eigen::Vector3f body(13.1f,31.3f,1.7f);
      pcl::PointCloud<PointType> walls;
      for (float x:{12.24f,13.96f})
        for (float y=29.f;y<=43.f;y+=.1f) for (float z=.3f;z<3.f;z+=.1f)
          walls.emplace_back(x,y,z);
      auto scan=make_shared<sensor_msgs::msg::PointCloud2>();pcl::toROSMsg(walls,*scan);
      auto odom=make_shared<nav_msgs::msg::Odometry>();odom->pose.pose.position.x=body.x();
      odom->pose.pose.position.y=body.y();odom->pose.pose.position.z=body.z();odom->pose.pose.orientation.w=1.;
      m.lidar_map_interface_->updateCloudMapOdometry(scan,odom);
      m.observed_air_.clear();m.visited_air_.clear();m.attempted_sightlines_.clear();m.survey_height_.reset();
      for (float y=31.f;y<40.f;y+=.1f)
        m.observed_air_.insert((Eigen::Vector3f(body.x(),y,body.z())/m.frtp_.cell_size_).array().floor().cast<int>());
      vector<Eigen::Vector3f> path;
      const Eigen::Vector3f far=body+Eigen::Vector3f(0,6,0);
      assert(m.connectObservedGrid(body,far,.5f*m.frtp_.cell_size_,.12,path));
      assert(m.certifyFlightGuide(path));
      path.clear();
      assert(m.reachableOpening(body,path));
      assert(path.back().y()>body.y()+2.f && m.certifyFlightGuide(path));
      m.visited_air_=m.observed_air_;
      const Eigen::Vector3f destination=body+Eigen::Vector3f(0,8,0);
      assert(m.reachableOpening(body,path,&destination));
      assert(path.back().y()>body.y()+3.f && m.certifyFlightGuide(path));
    }
    {
      const Eigen::Vector3f low(-27.5f,-21.5f,1.7f),high=low+2.1f*Eigen::Vector3f::UnitZ();
      const float cell=m.frtp_.cell_size_;
      m.surface_body_=low;
      m.survey_height_=low.z();
      m.flight_history_={low-Eigen::Vector3f::UnitX(),low};
      m.visited_air_.clear();
      m.observed_air_.clear();
      m.visited_air_.insert((low/cell).array().floor().cast<int>());
      const Eigen::Vector3i origin=(low/cell).array().floor().cast<int>();
      for (int x=-3;x<=10;++x) for (int y=-6;y<=6;++y) for (int z=-1;z<=10;++z)
        m.observed_air_.insert(origin+Eigen::Vector3i(x,y,z));
      assert(m.supportsElevatedOpening(low));
      assert(!m.supportsElevatedOpening(high));
      pcl::PointCloud<PointType> patches;
      for (float y=-1.3f;y<=1.3f;y+=.1f) for (float z=0;z<=.9f;z+=.1f)
        patches.emplace_back(high.x()+2.f,high.y()+y,high.z()+z);
      auto scan=make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(patches,*scan);
      auto odom=make_shared<nav_msgs::msg::Odometry>();
      odom->pose.pose.position.x=low.x();odom->pose.pose.position.y=low.y();
      odom->pose.pose.position.z=low.z();odom->pose.pose.orientation.w=1.;
      m.lidar_map_interface_->updateCloudMapOdometry(scan,odom);
      assert(m.supportsElevatedOpening(high));
      for (const auto &hit:patches)
        m.inspected_faces_[(hit.getVector3fMap()/(2.f*cell)).array().floor().cast<int>()]=15;
      assert(!m.supportsElevatedOpening(high));
    }
    {
      const Eigen::Vector3f body(20.2f,-16.2f,1.8f);
      m.surface_body_=body;m.survey_height_.reset();
      m.observed_air_.clear();m.visited_air_.clear();
      m.flight_history_={body-3.f*Eigen::Vector3f::UnitX(),body-Eigen::Vector3f::UnitX(),body};
      const float cell=m.frtp_.cell_size_;
      const Eigen::Vector3i origin=(body/cell).array().floor().cast<int>();
      for (int x=-8;x<35;++x) for(int y=-13;y<=13;++y) for(int z=-4;z<=4;++z)
        m.observed_air_.insert(origin+Eigen::Vector3i(x,y,z));
      auto forward=m.forwardObservation(body);
      assert(forward && forward->center_.x()>body.x()+6.f);
      const auto observation=m.observations_.at(forward.get());
      assert(m.certifyFlightGuide(observation->contour_approach_) && !m.unseenSupport(*observation).empty());
      // A sampled stance can still contribute to the tour when it has no
      // topology edge. Once the measured pass is flown the same sample expires.
      m.survey_height_=body.z();
      PointVector samples{PointType(forward->center_.x(),forward->center_.y(),forward->center_.z())};
      auto sampled=m.sampledSurveyPass(body,samples);
      assert(sampled && (sampled->center_-forward->center_).norm()<.001f);
      const auto &approach=m.observations_.at(sampled.get())->contour_approach_;
      assert(m.certifyFlightGuide(approach));
      const auto maximum=m.graph_->parallel_bubble_astar_->flight_max_z_;
      m.graph_->parallel_bubble_astar_->flight_max_z_=approach.back().z()-.1f;
      assert(!m.certifyFlightGuide(approach));
      m.graph_->parallel_bubble_astar_->flight_max_z_=maximum;
      m.visited_air_=m.observed_air_;
      assert(!m.sampledSurveyPass(body,samples));
      m.survey_height_.reset();
      assert(!m.forwardObservation(body));
      const Eigen::Vector3f dx=5.f*Eigen::Vector3f::UnitX(),dy=5.f*Eigen::Vector3f::UnitY();
      const vector<Eigen::Vector3f> last_edge{body,body+.45f*dx,body+.88f*dx};
      m.flight_history_={body+dx,body+dx+dy,body+dy,body};
      assert(m.closesBoundarySweep(last_edge));
      m.flight_history_={body+dx,body+dx-dy,body-dy,body};
      assert(!m.closesBoundarySweep(last_edge));
      m.flight_history_={body+dx,body+dx+dy,body+dy,body};
      m.observed_air_.clear();
      assert(!m.closesBoundarySweep(last_edge));

    }
    {
      const Eigen::Vector3f body(25.f,-12.f,1.8f);
      pcl::PointCloud<PointType> hits;
      for (float y=-23.f;y<=0.f;y+=.2f) for (float z=.2f;z<=3.f;z+=.2f)
        hits.emplace_back(body.x()+5.f,y,z);
      auto scan=make_shared<sensor_msgs::msg::PointCloud2>();pcl::toROSMsg(hits,*scan);
      auto odom=make_shared<nav_msgs::msg::Odometry>();
      odom->pose.pose.position.x=body.x();odom->pose.pose.position.y=body.y();
      odom->pose.pose.position.z=body.z();odom->pose.pose.orientation.w=1.;
      m.lidar_map_interface_->updateCloudMapOdometry(scan,odom);
      m.surface_body_=body;m.survey_height_=body.z();m.flight_history_.clear();
      for (int x=-12;x<=0;++x) m.flight_history_.push_back(body+x*Eigen::Vector3f::UnitX());
      m.observed_air_.clear();m.visited_air_.clear();
      const float cell=m.frtp_.cell_size_;
      const Eigen::Vector3i centre=(body/cell).array().floor().cast<int>();
      for (int x=-32;x<=15;++x) for(int y=-20;y<=20;++y) for(int z=-1;z<=1;++z)
        m.observed_air_.insert(centre+Eigen::Vector3i(x,y,z));
      auto turn=m.parallelObservation(body);
      assert(turn && turn->center_.y()<body.y()-2.f && turn->center_.x()<body.x());
      const auto straight_history=m.flight_history_;
      // A previous pass slightly offset inside the same visible strip has
      // already inspected it. Keep the other side available for the U-turn.
      for (int x=-12;x<=1;++x)
        m.flight_history_.insert(m.flight_history_.begin(),body+Eigen::Vector3f(x,-4.f,0.f));
      auto other=m.parallelObservation(body);
      assert(other && other->center_.y()>body.y()+2.f);
      m.flight_history_=straight_history;
      const auto &leg=*m.observations_.at(turn.get());
      assert(leg.contour_entry_ && m.certifyFlightGuide(leg.contour_approach_));
      assert(!m.parallelObservation(body,true));
      // A separating wall blocks both lateral cross sections behind the
      // vehicle, but its measured end permits a turn to the parallel band.
      pcl::PointCloud<PointType> divider;
      for (float x=body.x()-10.f;x<body.x()+.5f;x+=.2f) for(float z=.2f;z<=3.f;z+=.2f)
        divider.emplace_back(x,body.y()-.55f*m.frtp_.good_observation_trust_length_,z);
      pcl::toROSMsg(divider,*scan);
      m.lidar_map_interface_->updateCloudMapOdometry(scan,odom);
      auto separate=m.parallelObservation(body,true);
      assert(separate && separate->center_.y()<body.y()-2.f &&
          m.certifyFlightGuide(m.observations_.at(separate.get())->contour_approach_));
      // The far side of a divider can contain a lidar shadow. The first
      // observation must stop on the measured side of that shadow.
      const auto known_air=m.observed_air_;
      for (auto it=m.observed_air_.begin();it!=m.observed_air_.end();) {
        const Eigen::Vector3f p=(it->cast<float>().array()+.5f)*cell;
        if (p.y()<body.y()-3.f && p.x()>=body.x()-1.5f && p.x()<=body.x()+.7f)
          it=m.observed_air_.erase(it);
        else ++it;
      }
      auto mouth=m.parallelObservation(body,true);
      assert(mouth && !m.observations_.at(mouth.get())->contour_support_.empty());
      assert(m.certifyFlightGuide(m.observations_.at(mouth.get())->contour_approach_));
      m.observed_air_=known_air;
      m.visited_air_=m.observed_air_;
      assert(!m.parallelObservation(body));
      m.visited_air_.clear();
      for (auto it=m.observed_air_.begin();it!=m.observed_air_.end();)
        if (std::abs(it->y()-centre.y())>2) it=m.observed_air_.erase(it);else ++it;
      assert(!m.parallelObservation(body));
    }
    {
      const Eigen::Vector3f start(-14.f,-14.f,2.f),goal(-10.f,-14.f,2.f);
      m.surface_body_=start;
      m.graph_->parallel_bubble_astar_->safe_distance_=.8;
      m.observed_air_.clear();
      const float cell=m.frtp_.cell_size_;
      for (float x=start.x()-.5f;x<=goal.x()+.5f;x+=.5f*cell)
        for (float y=-15.f;y<=-13.f;y+=.5f*cell)
          // This fixture has one measured horizontal band around the wall.
          for (float z=start.z()-.2f;z<=start.z()+.2f;z+=.5f*cell)
            m.observed_air_.insert((Eigen::Vector3f(x,y,z)/cell).array().floor().cast<int>());
      // A protruding face interrupts the direct line. There is a narrow
      // measured strip on its other side, between the two fixed walls.
      pcl::PointCloud<PointType> cloud;
      for (float x=-14.5f;x<-9.5f;x+=.1f) for (float z=.8f;z<3.2f;z+=.1f) {
        cloud.emplace_back(x,-15.05f,z);
        cloud.emplace_back(x,std::abs(x+12.f)<.4f ? -13.29f:-12.8f,z);
      }
      auto msg=make_shared<sensor_msgs::msg::PointCloud2>();pcl::toROSMsg(cloud,*msg);
      auto odom=make_shared<nav_msgs::msg::Odometry>();
      odom->pose.pose.position.x=start.x();odom->pose.pose.position.y=start.y();
      odom->pose.pose.position.z=start.z();odom->pose.pose.orientation.w=1.;
      m.lidar_map_interface_->updateCloudMapOdometry(msg,odom);
      vector<Eigen::Vector3f> route;
      assert(!m.certifyFlightGuide({start,goal}));
      assert(m.connectObservedSections(start,goal,route));
      assert((route.front()-start).norm()<1.e-4f && (route.back()-goal).norm()<1.e-4f);
      assert(m.certifyFlightGuide(route));
      const int gap=int(std::floor(-12.f/cell));
      for (auto it=m.observed_air_.begin();it!=m.observed_air_.end();)
        if (it->x()==gap) it=m.observed_air_.erase(it);else ++it;
      assert(!m.connectObservedSections(start,goal,route));
    }

    {
      // On an upper storey, a floor supplies clearance but no wall-survey
      // objective. The same flight must still see a nearby shelf upright.
      const Eigen::Vector3f start(-29,23,7.5f),end(-23,23,7.5f);
      const float cell=m.frtp_.cell_size_;
      m.observed_air_.clear();m.visited_air_.clear();
      for (float x=-34;x<=-15;x+=.8f*cell) for (float y=15;y<=31;y+=.8f*cell)
        for (float z=4;z<=11;z+=.8f*cell)
          m.observed_air_.insert((Eigen::Vector3f(x,y,z)/cell).array().floor().cast<int>());
      pcl::PointCloud<PointType> cloud;
      for (float x=-31;x<=-16;x+=.15f) for (float y=17;y<=29;y+=.15f)
        cloud.emplace_back(x,y,5.8f);
      auto scan=make_shared<sensor_msgs::msg::PointCloud2>();
      auto odom=make_shared<nav_msgs::msg::Odometry>();
      odom->pose.pose.position.x=start.x();odom->pose.pose.position.y=start.y();
      odom->pose.pose.position.z=start.z();odom->pose.pose.orientation.w=1.;
      pcl::toROSMsg(cloud,*scan);m.lidar_map_interface_->updateCloudMapOdometry(scan,odom);
      const auto survey=[&] {return m.evaluateSurveyPath(start,Eigen::Vector2f::UnitX(),{start,end},6.f,true,false);};
      const auto saved_limit=m.frtp_.target_max_abs_normal_z_;
      m.frtp_.target_max_abs_normal_z_=1.f;
      assert(survey());
      m.frtp_.target_max_abs_normal_z_=.3f;
      assert(!survey());
      assert(!m.followsRightWall({end,start}));
      assert(!m.hasLateralSurveyGain(end));
      assert(!m.hasLateralSurveyGain(end,true)); // Earlier wall support cannot authorize this bare endpoint.
      vector<Eigen::Vector3f> recovery;
      assert(!m.reachableOpening(start,recovery));
      for (float x=-30;x<=-16;x+=.15f) for (float z=6;z<=9;z+=.15f)
        cloud.emplace_back(x,25.f,z);
      pcl::toROSMsg(cloud,*scan);m.lidar_map_interface_->updateCloudMapOdometry(scan,odom);
      assert(m.followsRightWall({end,start}));
      assert(m.hasLateralSurveyGain(end));
      assert(m.hasLateralSurveyGain(end,true));
      assert(!m.followsRightWall({start,end}));
      auto shelf=survey();
      assert(shelf && !m.observations_.at(shelf.get())->contour_support_.empty());
      for (const auto &hit:m.observations_.at(shelf.get())->contour_support_)
        assert(hit.kind==ObservationSample::Kind::WallFace && std::abs(hit.point.y()-25.f)<cell);

      // A far opening approached along this aisle is still straight locally.
      // A real near corner and a guide back out of the entrance have
      // different first edges even when their endpoints both lie to the right.
      const Eigen::Vector3f body(-28.3f,23.f,7.5f),ahead(-1,0,0),right(0,1,0);
      const float trust=m.frtp_.good_observation_trust_length_;
      m.frtp_.good_observation_trust_length_=4.5f;
      const auto classify=[&](const vector<Eigen::Vector3f> &guide,bool wall,bool opening) {
        return m.surveyDeparture(body,ahead.head<2>(),guide,wall,opening);
      };
      using Departure=FrontierManager::Departure;
      assert(classify({body,body+9.f*ahead,body+9.f*ahead+3.f*right},false,true)==Departure::Forward);
      assert(classify({body,body+5.f*ahead},true,false)==Departure::AlongWall);
      assert(classify({body,body+2.8f*ahead,body+2.8f*ahead+3.f*right},false,true)==Departure::RightOpening);
      assert(classify({body,body-3.f*ahead,body-3.f*ahead+3.f*right},false,true)==Departure::Backwards);
      m.frtp_.good_observation_trust_length_=trust;
      m.frtp_.target_max_abs_normal_z_=saved_limit;
    }

    {
      // Three independent test cases share a scan. A disproved foreground
      // hit clears, a thin post with current returns stays, and a hit outside
      // the measured rays is retained.
      auto sensor_map=std::make_shared<LIOInterface>();
      sensor_map->init(m.nh_);
      sensor_map->lp_->persistent_hit_range_=15.;
      sensor_map->lp_->sensor_offset_body_.setZero();
      const Eigen::Vector3f stale(3.f,0.f,2.f),post(3.f,1.25f,2.f),hidden(-3.f,0.f,2.f);
      const auto feed=[&](float y,bool first_scan) {
        pcl::PointCloud<PointType> cloud;
        if (first_scan) {
          cloud.emplace_back(stale.x(),stale.y(),stale.z());
          cloud.emplace_back(hidden.x(),hidden.y(),hidden.z());
        }
        cloud.emplace_back(post.x(),post.y(),post.z());
        for(int i=-45;i<=45;++i) for(int j=-24;j<=24;++j)
          cloud.emplace_back(5.8f,.055f*i,2.f+.055f*j);
        auto msg=make_shared<sensor_msgs::msg::PointCloud2>();pcl::toROSMsg(cloud,*msg);
        auto odom=make_shared<nav_msgs::msg::Odometry>();
        odom->pose.pose.position.y=y;odom->pose.pose.position.z=2.;odom->pose.pose.orientation.w=1.;
        sensor_map->updateCloudMapOdometry(msg,odom);
      };
      feed(0.f,true);
      // Both map instances are queried on this thread at exactly the same XYZ.
      const auto original_clearance=m.lidar_map_interface_->getDisToOcc(stale);
      const auto other_clearance=sensor_map->getDisToOcc(stale);
      assert(original_clearance!=other_clearance);
      assert(m.lidar_map_interface_->getDisToOcc(stale)==original_clearance);
      assert(sensor_map->getDisToOcc(stale)==other_clearance);
      for(int i=0;i<8;++i)feed(0.f,false);
      assert(sensor_map->getDisToOcc(stale)<.10);
      for(int i=1;i<=14;++i)feed(.03f*i,false);
      assert(sensor_map->getDisToOcc(stale)>.5);
      assert(sensor_map->getDisToOcc(post)<.10 && sensor_map->getDisToOcc(hidden)<.10);
    }

  }
};

int main(int argc,char **argv) {
  epic_ros::init(argc,argv,"surface_visibility_regression");epic_ros::NodeHandle nh;
  auto lio=make_shared<LIOInterface>();lio->init(nh);
  auto graph=make_shared<TopoGraph>();FrontierManager manager;manager.init(nh,lio,graph);
  SurfaceVisibilityRegression::run(manager);rclcpp::shutdown();
}
