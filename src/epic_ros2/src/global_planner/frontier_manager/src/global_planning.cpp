/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-07-12 10:30:16
 * @LastEditTime: 2024-07-12 21:35:39
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#include <epic_ros2/planning_workers.hpp>
#include <frontier_manager/frontier_manager.h>
#include <chrono>

class UF {
public:
  UF(int size) {
    father.resize(size);
    rank.resize(size, 0);
    for (int i = 0; i < size; ++i) {
      father[i] = i;
    }
  }

  int find(int x) {
    if (x != father[x]) {
      father[x] = find(father[x]); // Path compression
    }
    return father[x];
  }

  void connect(int x, int y) {
    int xx = find(x), yy = find(y);
    if (xx != yy) {
      if (rank[xx] < rank[yy]) {
        father[xx] = yy;
      } else if (rank[xx] > rank[yy]) {
        father[yy] = xx;
      } else {
        father[yy] = xx;
        rank[xx]++;
      }
    }
  }

private:
  vector<int> father;
  vector<int> rank;
};



void FrontierManager::generateTSPViewpoints(Eigen::Vector3f&center,  vector<TopoNode::Ptr> &viewpoints) {
  observations_.clear();
  viewpoints.clear();
  vector<TopoNode::Ptr> wall_views;
  if (surface_continuity_) {
    cluster_list_.remove_if([&](const auto &cluster){return observedCluster(*cluster);});
    appendSurfaceViewpoints(center,wall_views,true);
  }
  unordered_set<ClusterInfo::Ptr> revp_clusters_set; // (re)-generate viewpoints clusters
  vector<float> distance_odom2cluster;
  vector<ClusterInfo::Ptr> old_clusters_within_consideration;
  const double now = epic_ros::Time::now().toSec();
  for (auto &cluster : cluster_list_) {
    if (cluster->is_dormant_ || (surface_continuity_ &&
        (observedCluster(*cluster) || surveyedLowerSurface(*cluster,center))))
      continue;
    // A temporarily missing topology connection is not a permanently visited
    // surface. Re-evaluate after map growth; default 0 retains the old policy.
    if (!cluster->is_reachable_ && (vpp_.unreachable_retry_interval_ <= 0.f ||
        now-cluster->last_viewpoint_attempt_ < vpp_.unreachable_retry_interval_))
      continue;
    if (revp_clusters_set.count(cluster))
      continue;
    old_clusters_within_consideration.push_back(cluster);
    // float distance =
    // (center- cluster->center_).norm() + fabs(graph_->odom_node_->center_.z() - cluster->center_.z()) * 0.5;
    float distance = graph_->estimateRoughDistance(cluster->center_, cluster->odom_id_);
    distance_odom2cluster.push_back(distance);
  }

  vector<int> idx;
  for (int i = 0; i < distance_odom2cluster.size(); i++) {
    idx.push_back(i);
  }

  sort(idx.begin(), idx.end(), [&](int a, int b) { return distance_odom2cluster[a] < distance_odom2cluster[b]; });

  int consider_range = min(vpp_.local_tsp_size_, (int)idx.size());
  // cout << "old_clusters_within_consideration num: " << consider_range << endl;
  for (int i = 0; i < consider_range; i++) {
    old_clusters_within_consideration[idx[i]]->is_new_cluster_ = false;
    revp_clusters_set.insert(old_clusters_within_consideration[idx[i]]);
  }
  if (surface_continuity_) {
    // Retain the physical support, not a waypoint. A disappeared cluster is
    // resampled and ray-tested exactly like live clusters before it can fly.
    // A saved surface is only a retry of a live frontier. Recheck the
    // voxel labels so a retired patch cannot keep attracting new viewpoints.
    pending_surfaces_.erase(std::remove_if(pending_surfaces_.begin(),pending_surfaces_.end(),
      [&](const auto &item) {
        if (observedCluster(*item)) return true;
        const auto live=std::count_if(item->cells_.begin(),item->cells_.end(),[&](const auto &cell) {
          const auto label=get_state(cell);
          return label==FRONTIER_DIS || label==FRONTIER_DIR;
        });
        return live<std::max<size_t>(1,(item->cells_.size()+4)/5);
      }),pending_surfaces_.end());
    const size_t trials=std::min<size_t>(2,pending_surfaces_.size());
    for (size_t i=0;i<trials;++i) {
      next_pending_surface_%=pending_surfaces_.size();
      auto saved=pending_surfaces_[next_pending_surface_++];
      const bool represented=std::any_of(revp_clusters_set.begin(),revp_clusters_set.end(),
        [&](const auto &live){return (live->center_-saved->center_).norm()<2.f*frtp_.cell_size_ &&
                                    live->normal_.dot(saved->normal_)>.8f;});
      if (!represented) {
        saved->is_dormant_=false;
        revp_clusters_set.insert(saved);
      }
    }
  }
  // 附近的+新生成的
  vector<ClusterInfo::Ptr> revp_clusters_vec; // revp: regenerate viewpoint
  revp_clusters_vec.insert(revp_clusters_vec.end(), revp_clusters_set.begin(), revp_clusters_set.end());
  epic_ros::Time t1 = epic_ros::Time::now();
  omp_set_num_threads(epic_ros::planningWorkerCount(4));
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &cluster : revp_clusters_vec) {
    initClusterViewpoints(cluster);
  }
  epic_ros::Time t2 = epic_ros::Time::now();
  // cout << "init cluster viewpoint cost: " << (t2 - t1).toSec() * 1000 << "ms" << endl;

  PointVector sampled_viewpoints;
  PointVector vp_centers;
  for (auto &cls : revp_clusters_vec) {
    for (auto &vpc : cls->vp_clusters_) {
      sampled_viewpoints.insert(sampled_viewpoints.end(), vpc.vps_.begin(), vpc.vps_.end());
      vp_centers.emplace_back(vpc.center_.x(), vpc.center_.y(), vpc.center_.z());
    }
  }
  for (const auto &wall:wall_views)
    sampled_viewpoints.emplace_back(wall->center_.x(),wall->center_.y(),wall->center_.z());
  viz_point(vp_centers, "viewpoint_centers");

  removeUnreachableViewpoints(revp_clusters_vec);
  vector<ClusterInfo::Ptr> clusters_can_be_searched_;
  for (auto &cluster : revp_clusters_vec) {
    if (cluster->is_reachable_)
      clusters_can_be_searched_.push_back(cluster);
  }

  epic_ros::Time t3 = epic_ros::Time::now();
  // cout << "remove unreachable cluster cost: " << (t3 - t2).toSec() * 1000 << "ms" << endl;
  // cout << "revp cluster size: " << revp_clusters_vec.size() << endl;
  // cout << "reab cluster size: " << clusters_can_be_searched_.size() << endl;
  // updateHalfSpaces(clusters_can_be_searched_);
  vector<ClusterInfo::Ptr> tsp_clusters;
  mutex mtx;
  unordered_set<int> cluster2remove;
  omp_set_num_threads(epic_ros::planningWorkerCount(6));
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 0; i < clusters_can_be_searched_.size(); i++) {
    auto cluster = clusters_can_be_searched_[i];
    selectBestViewpoint(cluster);
    if (!cluster->is_reachable_)
      continue;
    mtx.lock();
    tsp_clusters.push_back(cluster);
    if (cluster->is_dormant_) {
      cluster2remove.insert(cluster->id_);
    }
    mtx.unlock();
  }
  // 飞到但看不到，说明odom漂了，这篇工作不处理，直接跳过
  cluster_list_.remove_if([&](ClusterInfo::Ptr cluster) {
    bool remove = cluster2remove.count(cluster->id_);
    if (remove) {
      for (auto &cell : cluster->cells_) {
        Eigen::Vector3i idx;
        pos2idx(cell, idx);
        ByteArrayRaw bytes;
        idx2bytes(idx, bytes);
        frtd_.label_map_[bytes] = DENSE;
      }
    }
    return remove;
  });
  epic_ros::Time t4 = epic_ros::Time::now();
  // cout << "select best viewpoint cost: " << (t4 - t3).toSec() * 1000 << "ms" << endl;
  // 重新topK
  vector<float> distance2odom2;
  vector<int> idx2;
  for (int i = 0; i < tsp_clusters.size(); i++) {
    float distance = tsp_clusters[i]->distance_;
    distance2odom2.push_back(distance);
    idx2.push_back(i);
  }
  sort(idx2.begin(), idx2.end(), [&](int a, int b) { return distance2odom2[a] < distance2odom2[b]; });
  float mean_distance = accumulate(distance2odom2.begin(), distance2odom2.end(), 0.0) / distance2odom2.size();
  viewpoints.clear();
  for (int i = 0; i < (int)idx2.size(); i++) {
    // 剔除异常值
    // if (i > (int)(idx2.size() / 2.0) && distance2odom2[idx2[i]] > mean_distance * 5.0)
    //   break;
    if (surface_continuity_) {
      const auto &cluster=tsp_clusters[idx2[i]];
      if (!observedAir(cluster->best_vp_) || observedCluster(*cluster) || unseenSupport(*cluster).empty()) continue;
      const bool saved=std::any_of(pending_surfaces_.begin(),pending_surfaces_.end(),
        [&](const auto &item){return (item->center_-cluster->center_).norm()<2.f*frtp_.cell_size_ &&
                                    item->normal_.dot(cluster->normal_)>.8f;});
      if (!saved) pending_surfaces_.push_back(std::make_shared<ClusterInfo>(*cluster));
    }
    TopoNode::Ptr vp_node = make_shared<TopoNode>();
    vp_node->is_viewpoint_ = true;
    vp_node->center_ = tsp_clusters[idx2[i]]->best_vp_;
    vp_node->yaw_ = tsp_clusters[idx2[i]]->best_vp_yaw_;
    observations_.emplace(vp_node.get(), tsp_clusters[idx2[i]]);
    viewpoints.push_back(vp_node);
  }
  if (surface_continuity_ && survey_height_) {
    const auto direction=recentFlightDirection(center);
    const float radius=graph_->bubble_min_radius_,trust=frtp_.good_observation_trust_length_;
    if (direction) {
      bool covering=false;
      for (const auto &view:wall_views) {
        const auto &observation=*observations_.at(view.get());
        if (observation.right_hand_opening_ || observation.contour_entry_) {covering=true;break;}
        Eigen::Vector2f far=(view->center_-center).head<2>();
        for (const auto &p:observation.contour_preview_) {
          if (std::abs(p.z()-center.z())<radius && (p-center).head<2>().dot(*direction)>far.dot(*direction))
            far=(p-center).head<2>();
        }
        const float along=far.dot(*direction);
        const float right=far.x()*direction->y()-far.y()*direction->x();
        if ((along>=trust && along>.7f*far.norm()) ||
            (along>=0.f && right>2.f*radius && observation.fresh_contour_length_>2.f*radius)) {
          covering=true;break;
        }
      }
      if (!covering) {
        if (auto advance=rollingSurveyView(center,*direction)) {
          perimeter_target_=advance->center_;
          wall_views.insert(wall_views.begin(),advance);
          sampled_viewpoints.emplace_back(advance->center_.x(),advance->center_.y(),advance->center_.z());
        }
      }
    }
  }
  if (surface_continuity_ && wall_views.empty() && survey_height_) {
    // A local outline can run out where a previously observed approach
    // opens onto uninspected surfaces. Add at most one supported continuation.
    const auto started=std::chrono::steady_clock::now();
    auto onward=sampledSurveyPass(center,sampled_viewpoints);
    if (!onward) {
      for (const auto &candidate:viewpoints) {
        if (std::chrono::steady_clock::now()-started>std::chrono::milliseconds(40)) break;
        onward=surveyApproach(center,candidate->center_);
        if (onward) break;
      }
    }
    if (onward) {
      perimeter_target_=onward->center_;
      wall_views.push_back(onward);
      sampled_viewpoints.emplace_back(onward->center_.x(),onward->center_.y(),onward->center_.z());
    }
  }
  if (surface_continuity_ && survey_height_) {
    viewpoints.erase(std::remove_if(viewpoints.begin(),viewpoints.end(),[&](const auto &vp) {
      if (!aboveSameSurveyFloor(vp->center_)) return false;
      observations_.erase(vp.get());
      return true;
    }),viewpoints.end());
  }
  if (surface_continuity_ && currentSurveyBand(center)) {
    const float radius=graph_->bubble_min_radius_;
    const auto climbs=[&](const auto &v){return v->center_.z()>center.z()+radius;};
    const auto productive=[&](const auto &v) {
      return std::abs(v->center_.z()-center.z())<=radius &&
          !observations_.at(v.get())->contour_approach_.empty() && !visitedStance(v->center_) &&
          !unseenSupport(*observations_.at(v.get())).empty();
    };
    if ((std::any_of(viewpoints.begin(),viewpoints.end(),climbs) || std::any_of(wall_views.begin(),wall_views.end(),climbs)) &&
        !std::any_of(wall_views.begin(),wall_views.end(),productive)) {
      vector<Eigen::Vector3f> approach;
      if (pendingLevelGuide(center,approach)) {
        if (auto view=boundaryObservation(std::move(approach))) {
          perimeter_target_=view->center_;
          wall_views.insert(wall_views.begin(),view);
          sampled_viewpoints.emplace_back(view->center_.x(),view->center_.y(),view->center_.z());
        }
      }
    }
  }
  // A native or detached viewpoint can be reachable by a path that passes
  // a measured right fork and then turns left. Offer that fork to the ATSP
  // even when the local wall sampler has only returned the left endpoint.
  if (surface_continuity_ && !std::any_of(wall_views.begin(),wall_views.end(),[&](const auto &v) {
        return observations_.at(v.get())->right_hand_opening_;
      })) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(45);
    const float radius=graph_->bubble_min_radius_;
    const auto find_branch=[&](const vector<TopoNode::Ptr> &candidates)->TopoNode::Ptr {
      for (const auto &node:candidates) {
        if (std::chrono::steady_clock::now()>=deadline) break;
        const Eigen::Vector3f delta=node->center_-center;
        if (std::abs(delta.z())>radius || delta.norm()<3.f*radius || delta.norm()>frtp_.update_length_) continue;
        auto guide=observations_.at(node.get())->contour_approach_;
        if (guide.empty() && !connectObserved(center,node->center_,.008,guide)) continue;
        if (guide.size()<2) continue;
        guide.front()=center;
        const auto entry=preferRightBranch(center,guide,false);
        if (!entry) continue;
        if (auto right=boundaryObservation(std::move(guide))) {
          auto &observation=*observations_.at(right.get());
          observation.right_hand_opening_=true;
          observation.contour_entry_=entry;
          return right;
        }
      }
      return {};
    };
    auto branch=find_branch(wall_views);
    if (!branch) branch=find_branch(viewpoints);
    if (branch) {
      perimeter_target_=branch->center_;
      wall_views.insert(wall_views.begin(),branch);
      sampled_viewpoints.emplace_back(branch->center_.x(),branch->center_.y(),branch->center_.z());
    }
  }
  if (surface_continuity_ && perimeter_target_) {
    if (auto onward=continueIssuedRightTurn(center,*perimeter_target_)) {
      perimeter_target_=onward->center_;
      wall_views.insert(wall_views.begin(),onward);
      sampled_viewpoints.emplace_back(onward->center_.x(),onward->center_.y(),onward->center_.z());
    }
  }
  if (surface_continuity_ && frtp_.target_max_abs_normal_z_<1.f) {
    if (auto exterior=exteriorViewpoint(center)) {
      perimeter_target_=exterior->center_;
      wall_views.insert(wall_views.begin(),exterior);
    }
  }
  viewpoints.insert(viewpoints.end(),wall_views.begin(),wall_views.end());
  if (surface_continuity_ && frtp_.target_max_abs_normal_z_<1.f) {
    // Evidence along a sweep can belong to its departure. Check the actual
    // endpoint for both native and generated observations before issuing it.
    viewpoints.erase(std::remove_if(viewpoints.begin(),viewpoints.end(),[&](const auto &node) {
      const auto &view=*observations_.at(node.get());
      const bool wall=view.follows_local_outline_ || !view.visible_cells_.empty() ||
          std::any_of(view.contour_support_.begin(),view.contour_support_.end(),[](const auto &sample) {
            return sample.kind==ObservationSample::Kind::WallFace;
          });
      if (hasLateralSurveyGain(node->center_,wall)) return false;
      // The first viewpoint can be a transit stance through the bend. Keep
      // a local outline if its checked continuation still inspects new wall.
      const auto &preview=view.contour_preview_;
      const bool checked_preview=view.follows_local_outline_ && preview.size()>1 &&
          (preview.front()-node->center_).norm()<.01f &&
          std::any_of(preview.begin()+1,preview.end(),[&](const auto &point) {
            return !visitedStance(point) && hasLateralSurveyGain(point,true);
          }) && certifyFlightGuide(preview);
      if (checked_preview) {
        ROS_INFO("Kept measured wall turn for fresh observations beyond its first stance");
        return false;
      }
      if (std::getenv("EPIC_TRACE_WALL_CHOICE") && view.follows_local_outline_)
        ROS_INFO("Rejected wall outline at %.2f %.2f: no lateral gain at the selected stance or checked preview",
                 node->center_.x(),node->center_.y());
      if (perimeter_target_ && (*perimeter_target_-node->center_).squaredNorm()<1.e-6f)
        perimeter_target_.reset();
      return true;
    }),viewpoints.end());
  } else {
    viz_point(sampled_viewpoints, "sampled_viewpoints");
  }
  if (surface_continuity_ && frtp_.target_max_abs_normal_z_>=1.f) {
    vector<Eigen::Vector3f> shown;
    for (const auto &node:viewpoints)
      if (observations_.at(node.get())->close_surface_sample_) shown.push_back(node->center_);
    viz_point(shown,"inspection_viewpoints");
  }
  ROS_INFO("vp cluster cost: %fms  ,remove unreachable cost: %fms, select vp cost: %fms", (t2 - t1).toSec() * 1000, (t3 - t2).toSec() * 1000,
           (t4 - t3).toSec() * 1000);
}

void FrontierManager::publishSurveyViewpoints(const vector<TopoNode::Ptr> &viewpoints) {
  if (!surface_continuity_ || frtp_.target_max_abs_normal_z_>=1.f) return;
  vector<Eigen::Vector3f> samples,inspections;
  for (const auto &node:viewpoints) {
    samples.push_back(node->center_);
    if (observations_.at(node.get())->close_surface_sample_) inspections.push_back(node->center_);
  }
  viz_point(samples,"sampled_viewpoints");
  viz_point(inspections,"inspection_viewpoints");
}
