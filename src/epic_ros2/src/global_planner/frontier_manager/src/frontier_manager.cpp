/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-04-14 21:44:58
 * @LastEditTime: 2024-04-15 13:30:53
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#include <frontier_manager/frontier_manager.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <visualization_msgs/msg/marker_array.hpp>
size_t ByteArrayRaw::size = 0;
void FrontierManager::init(epic_ros::NodeHandle &nh, LIOInterface::Ptr &lio_interface,
                           TopoGraph::Ptr graph) {
  nh_ = nh;
  graph_ = graph;
  lidar_map_interface_ = lio_interface;
  // Inspection target policy is separate from the full 3-D collision map and
  // topology ROI. Defaults preserve the original EPIC mission.
  nh.param("FrontierManager/floor_range_frontiers",frtp_.floor_range_frontiers_,false);
  nh.param("FrontierManager/ascending_floor_frontiers",frtp_.ascending_floor_frontiers_,false);
  nh.param("ViewpointManager/walls_at_current_height",frtp_.wall_viewpoints_at_current_height_,false);
  nh.param("FrontierManager/floor_viewpoint_clearance",frtp_.floor_viewpoint_clearance_,1.7f);
  nh.param("FrontierManager/max_frontier_support_distance",frtp_.max_frontier_support_distance_,-1.f);
  nh.param("FrontierManager/debug_viewpoint_rejections",debug_viewpoints_,false);
  nh.param("FrontierManager/target_min_z", frtp_.target_min_z_, -1.0e6);
  nh.param("FrontierManager/target_max_z", frtp_.target_max_z_, 1.0e6);
  nh.param("FrontierManager/target_max_abs_normal_z", frtp_.target_max_abs_normal_z_, 1.0);
  nh.param("ViewpointManager/unreachable_retry_interval",vpp_.unreachable_retry_interval_,0.);
  nh.param("ViewpointManager/fixed_height_enabled", vpp_.fixed_height_enabled_, false);
  nh.param("ViewpointManager/fixed_height", vpp_.fixed_height_, 1.75);
  nh.param("ViewpointManager/reached_position_tolerance", vpp_.reached_position_tolerance_, .01);
  nh.param("ViewpointManager/reached_yaw_tolerance", vpp_.reached_yaw_tolerance_, .01);
  if (frtp_.target_min_z_>frtp_.target_max_z_ ||
      frtp_.target_max_abs_normal_z_<0.f || frtp_.target_max_abs_normal_z_>1.f ||
      !std::isfinite(vpp_.fixed_height_) || vpp_.reached_position_tolerance_<=0.f ||
      vpp_.reached_yaw_tolerance_<=0.f || vpp_.unreachable_retry_interval_<0.f)
    throw std::invalid_argument("Invalid inspection target/viewpoint parameters");

  nh.getParam("FrontierManager/cell_size", frtp_.cell_size_);
  frtp_.inv_cell_size_ = 1 / frtp_.cell_size_;
  nh.getParam("FrontierManager/noise_cell_range", frtp_.noise_cell_range_);
  nh.getParam("FrontierManager/good_observation_direction_score",
              frtp_.good_observation_direction_score_);
  nh.getParam("FrontierManager/good_observation_trust_length",
              frtp_.good_observation_trust_length_);
  nh.getParam("FrontierManager/good_observation_force_trust_length",
              frtp_.good_observation_force_trust_length_);
  nh.getParam("FrontierManager/update_length", frtp_.update_length_);
  nh.getParam("FrontierManager/view_frt", frtp_.view_frt_);
  nh.getParam("FrontierManager/view_cluster", frtp_.view_cluster_);

  nh.getParam("FrontierManager/cluster_min_radius", frtp_.cluster_min_radius_);
  nh.getParam("FrontierManager/cluster_min_size", frtp_.cluster_min_size_);
  nh.getParam("FrontierManager/cluster_max_size", frtp_.cluster_radius_);
  nh.getParam("FrontierManager/cluster_direction_radius",
              frtp_.cluster_direction_radius_);
  nh.getParam("FrontierManager/cluster_minmum_point_num",
              frtp_.cluster_minmum_point_num_);

  // frt_cluster_ptr_.reset(new FrontierCluster);
  // frt_cluster_ptr_->init(nh);

  nh.getParam("ViewpointManager/sample_pillar_height_layer_num",
              vpp_.sample_pillar_height_layer_num_);
  nh.getParam("ViewpointManager/sample_pillar_radius_layer_num",
              vpp_.sample_pillar_radius_layer_num_);
  nh.getParam("ViewpointManager/sample_pillar_circle_sample_num",
              vpp_.sample_pillar_circle_sample_num_);
  nh.getParam("ViewpointManager/sample_pillar_max_height",
              vpp_.sample_pillar_max_height_);
  nh.getParam("ViewpointManager/sample_pillar_min_height",
              vpp_.sample_pillar_min_height_);
  nh.getParam("ViewpointManager/sample_pillar_min_radius",
              vpp_.sample_pillar_min_radius_);
  nh.getParam("ViewpointManager/sample_pillar_max_radius",
              vpp_.sample_pillar_max_radius_);

  nh.getParam("ViewpointManager/consider_range", vpp_.consider_range_);
  nh.getParam("ViewpointManager/global_recluster_size",
              vpp_.global_recluster_size_);
  nh.getParam("ViewpointManager/local_tsp_size", vpp_.local_tsp_size_);

  nh.getParam("lidar_perception/fov_viewpoint_up", vpp_.fov_up_);
  nh.getParam("lidar_perception/lidar_pitch", vpp_.lidar_pitch_);
  nh.getParam("lidar_perception/fov_viewpoint_down", vpp_.fov_down_);

  vpp_.view_direction_range_ = cos(vpp_.view_direction_range_ * M_PI / 180.0);
  vpp_.fov_up_ = vpp_.fov_up_ * M_PI / 180.0;
  vpp_.fov_down_ = vpp_.fov_down_ * M_PI / 180.0;
  frtp_.map_min_ =
      lidar_map_interface_->lp_->global_map_min_boundary_.cast<float>();
  frtp_.map_max_ =
      lidar_map_interface_->lp_->global_map_max_boundary_.cast<float>();
  frtp_.cell_max_cnt_ =
      ((frtp_.map_max_ - frtp_.map_min_).array() / frtp_.cell_size_)
          .cast<int>()
          .matrix() +
      Eigen::Vector3i::Ones();
  frtp_.bits_need_.x() = std::ceil(std::log2(frtp_.cell_max_cnt_.x()));
  frtp_.bits_need_.y() = std::ceil(std::log2(frtp_.cell_max_cnt_.y()));
  frtp_.bits_need_.z() = std::ceil(std::log2(frtp_.cell_max_cnt_.z()));
  frtp_.idx_byte_size_ =
      (frtp_.bits_need_.x() + frtp_.bits_need_.y() + frtp_.bits_need_.z() + 7) /
      8;

  float start_degree = 0;
  float degree_step = 2 * M_PI / vpp_.sample_pillar_circle_sample_num_;
  float start_degree_step =
      degree_step / (float)vpp_.sample_pillar_height_layer_num_;
  float height_step =
      (vpp_.sample_pillar_max_height_ - vpp_.sample_pillar_min_height_) /
      vpp_.sample_pillar_height_layer_num_;
  float radius_step =
      (vpp_.sample_pillar_max_radius_ - vpp_.sample_pillar_min_radius_) /
      vpp_.sample_pillar_radius_layer_num_;
  for (float height = vpp_.sample_pillar_min_height_;
       height <= vpp_.sample_pillar_max_height_ - 1e-3; height += height_step) {
    for (float radius = vpp_.sample_pillar_min_radius_;
         radius <= vpp_.sample_pillar_max_radius_ - 1e-3;
         radius += radius_step) {
      start_degree += start_degree_step;
      for (float degree = start_degree;
           degree <= start_degree + 2 * M_PI - 1e-6; degree += degree_step) {
        Eigen::Vector3f vp(radius * cos(degree), radius * sin(degree), height);
        origin_viewpoints_.push_back(vp);
      }
    }
  }
  frtd_ = FrontierData(frtp_.idx_byte_size_);
  frtd_.label_map_.max_load_factor(1.5);
}

void FrontierManager::pos2idx(const PointType &pt, Eigen::Vector3i &idx) {
  //
  idx = ((pt.getVector3fMap() - frtp_.map_min_) * frtp_.inv_cell_size_)
            .array()
            .floor()
            .cast<int>();
}

void FrontierManager::pos2idx(const Eigen::Vector3f &pt, Eigen::Vector3i &idx) {
  //
  idx = ((pt - frtp_.map_min_) * frtp_.inv_cell_size_)
            .array()
            .floor()
            .cast<int>();
}

void FrontierManager::idx2bytes(const Eigen::Vector3i &idx,
                                ByteArrayRaw &bytes) {
  // 无需resize，因为ByteArrayRaw在构造时已分配好固定大小
  uint64_t value = (static_cast<uint64_t>(idx.x())
                    << (frtp_.bits_need_.y() + frtp_.bits_need_.z())) |
                   (static_cast<uint64_t>(idx.y()) << frtp_.bits_need_.z()) |
                   static_cast<uint64_t>(idx.z());

  for (int i = 0; i < frtp_.idx_byte_size_; ++i) {
    bytes.data[i] = static_cast<uint8_t>(value >> (i * 8));
  }
}

void FrontierManager::bytes2pos(const ByteArrayRaw &bytes, PointType &pt) {
  Eigen::Vector3i idx;

  uint64_t value = 0;
  for (int i = frtp_.idx_byte_size_ - 1; i >= 0; --i) {
    value = (value << 8) | static_cast<uint64_t>(bytes.data[i]);
  }

  idx.z() = static_cast<int>(value & ((1 << frtp_.bits_need_.z()) - 1));
  value >>= frtp_.bits_need_.z();

  idx.y() = static_cast<int>(value & ((1 << frtp_.bits_need_.y()) - 1));
  value >>= frtp_.bits_need_.y();

  idx.x() = static_cast<int>(value);

  Eigen::Vector3f pt_v3f =
      (idx.cast<float>() + 0.5 * Eigen::Vector3f::Ones()) * frtp_.cell_size_ +
      frtp_.map_min_;
  pt.x = pt_v3f.x();
  pt.y = pt_v3f.y();
  pt.z = pt_v3f.z();
}

void FrontierManager::pos2bytes(const PointType &pt, ByteArrayRaw &bytes) {
  Eigen::Vector3i idx =
      ((pt.getVector3fMap() - frtp_.map_min_) * frtp_.inv_cell_size_)
          .array()
          .floor()
          .cast<int>();
  idx2bytes(idx, bytes);
}

CELL_STATE FrontierManager::get_state(const PointType &pt) {
  Eigen::Vector3i idx;
  pos2idx(pt, idx);
  return get_state(idx);
}

CELL_STATE FrontierManager::get_state(const Eigen::Vector3i &idx) {
  ByteArrayRaw bytes;
  idx2bytes(idx, bytes);
  if (frtd_.label_map_.find(bytes) == frtd_.label_map_.end())
    return UNKNOWN;
  else
    return (CELL_STATE)frtd_.label_map_[bytes];
}

void FrontierManager::get_cells_2_update(
    const PointVector &points, vector<Eigen::Vector3i> &cells_2_update) {
  cells_2_update.clear();
  std::unordered_set<Eigen::Vector3i, Vector3i_Hash> cells_2_update_set;
  std::unordered_set<Eigen::Vector3i, Vector3i_Hash> updated;
  Eigen::Vector3f lidar_position =
      lidar_map_interface_->ld_->lidar_pose_.cast<float>();
  for (auto &pt : points) {
    if (!lidar_map_interface_->IsInBox(pt))
      continue;
    if ((pt.getVector3fMap() -
         lidar_map_interface_->ld_->lidar_pose_.cast<float>())
            .norm() > frtp_.update_length_)
      continue;
    Eigen::Vector3i idx;
    pos2idx(pt, idx);
    if (updated.count(idx))
      continue;
    cells_2_update_set.insert(idx);
    updated.insert(idx);
    if (is_gap_point(pt) || is_fov_edge(pt)) {
      continue;
    }
    // 下面这块是为了去除噪声，也可以改成raycast
    for (int i = -frtp_.noise_cell_range_; i <= frtp_.noise_cell_range_; i++)
      for (int j = -frtp_.noise_cell_range_; j <= frtp_.noise_cell_range_; j++)
        for (int k = -frtp_.noise_cell_range_; k <= frtp_.noise_cell_range_;
             k++) {
          if (i == 0 && j == 0 && k == 0)
            continue;
          Eigen::Vector3i cell = idx + Eigen::Vector3i(i, j, k);
          if (cells_2_update_set.count(cell))
            continue;
          ByteArrayRaw bytes;
          idx2bytes(cell, bytes);
          if (!frtd_.label_map_.count(bytes))
            continue;
          if (frtd_.label_map_[bytes] == DENSE)
            continue;
          cells_2_update_set.insert(cell);
        }
  }
  cells_2_update.insert(cells_2_update.end(), cells_2_update_set.begin(),
                        cells_2_update_set.end());
}

void FrontierManager::get_pts_in_cells(
    const vector<Eigen::Vector3i> &cells_2_update,
    vector<PointVector> &pts_inside) {
  epic_ros::Time t1 = epic_ros::Time::now();
  pts_inside.clear();
  pts_inside.resize(cells_2_update.size());
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto i = 0; i < cells_2_update.size(); i++) {
    if (get_state(cells_2_update[i]) == DENSE)
      continue;
    PointType pt;
    idx2pos(cells_2_update[i], pt);
    vector<float> t;
    lidar_map_interface_->KNN(pt, 15, pts_inside[i], t);
  }
  // ROS_INFO("get_pts_in_cells time cost: %f", (epic_ros::Time::now() - t1).toSec()
  // * 1000.0);
}

bool FrontierManager::is_gap_point(const PointType &pt) {

  return frtd_.is_gap_[surface_pos2idx(pt)];
  // return false;
}

void FrontierManager::update_lidar_pt_gap(const vector<float> &depth) {

  frtd_.is_gap_ = vector<bool>(20000, false);
  auto viz_img = [&](cv::Mat img, string name) {
    cv::Mat image_8u, upsampled;
    cv::normalize(img, image_8u, 0, 255, cv::NORM_MINMAX);
    image_8u.convertTo(image_8u, CV_8UC1);
    cv::resize(image_8u, upsampled, cv::Size(400, 800));
    cv::imshow(name, upsampled);
    cv::waitKey(1);
  };
  Eigen::Vector3f lidar_position =
      lidar_map_interface_->ld_->lidar_pose_.cast<float>();
  frtd_.direction_score_ = vector<float>(20000, 0.0f);
  static vector<Eigen::Vector2i> diff_lis{{-1, 0}, {0, -1}, {0, 1}, {1, 0}};
  static vector<Eigen::Vector2i> diff_lis2{{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                                           {0, 1},   {1, -1}, {1, 0},  {1, 1}};
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 0; i < 100; i++) {
    for (int j = 0; j < 200; j++) {
      float dis1 = depth[i * 200 + j];
      for (auto &diff : diff_lis2) {
        if (!(i + diff[0] >= 0 && i + diff[0] < 100 && j + diff[1] >= 0 &&
              j + diff[1] < 200))
          continue;
        float dis2 = depth[(i + diff[0]) * 200 + (j + diff[1])];
        if (dis1 < 0 || dis2 < 0) {
          frtd_.is_gap_[i * 200 + j] = true;
          break;
        }
        float score = dis1 / dis2;
        if (score > 1.0)
          score = 1 / score;
        // float score = dis2 / dis1;
        frtd_.direction_score_[i * 200 + j] =
            frtd_.direction_score_[i * 200 + j] <= 1e-6
                ? score
                : min(frtd_.direction_score_[i * 200 + j], score);
        if (frtd_.direction_score_[i * 200 + j] <
            frtp_.good_observation_direction_score_) {
          frtd_.is_gap_[i * 200 + j] = true;
          break;
          // if (fabs(dis1 - dis2) > 0.5) {
          //   is_gap_[i * 200 + j] = true;
          //   break;
        }
      }
    }
  }
  vector<bool> is_gap_tmp = frtd_.is_gap_;
  for (int i = 0; i < 100; i++) {
    for (int j = 0; j < 200; j++) {
      if (is_gap_tmp[i * 200 + j]) {
        for (auto &diff : diff_lis2) {
          if (!(i + diff[0] >= 0 && i + diff[0] < 100 && j + diff[1] >= 0 &&
                j + diff[1] < 200)) {
            continue;
          }
          frtd_.is_gap_[(i + diff[0]) * 200 + (j + diff[1])] = true;
        }
      }
    }
  }
  epic_ros::Time t4 = epic_ros::Time::now();

  cv::Mat img_gap = cv::Mat::zeros(100, 200, CV_8UC1);
  cv::Mat img_depth = cv::Mat::zeros(100, 200, CV_32FC1);
  for (int i = 0; i < 100; i++)
    for (int j = 0; j < 200; j++) {
      img_gap.at<uchar>(i, j) = frtd_.is_gap_[i * 200 + j] ? 255 : 0;
      img_depth.at<float>(i, j) = depth[i * 200 + j];
    }
}

void FrontierManager::cluster_frts(const PointVector &frt_new,
                                   vector<ClusterInfo::Ptr> &new_clusters,
                                   vector<int> &cluster_removed) {
  cluster_removed.clear();
  PointVector frts2cluster;
  // static int frt_cluser_id = 0;
  // cout << "SF_list size: " << frt_cluster_ptr_->SF_list.size() << endl;
  vector<Eigen::Vector3f> frts_norm;
  PointVector unsupported;
  vector<Eigen::Vector3f> unsupported_norms;
  unordered_set<ByteArrayRaw, ByteArrayRawHasher> queued;
  auto enqueue = [&](const PointType &pt, const Eigen::Vector3f &fallback) {
    if (get_state(pt) != FRONTIER_DIS && get_state(pt) != FRONTIER_DIR) return;
    ByteArrayRaw bytes; pos2bytes(pt, bytes);
    if (!queued.insert(bytes).second) return;
    // A point-based surface frontier needs measured support. Neighbour-cell
    // updates can leave labels away from the actual surface (e.g. under a
    // floor). This is only a target filter: never mark such a label observed.
    if(frtp_.max_frontier_support_distance_>0.f &&
        lidar_map_interface_->getDisToOcc(pt)>frtp_.max_frontier_support_distance_) {
      unsupported.push_back(pt);unsupported_norms.push_back(fallback);return;
    }
    auto found = frtd_.frt_map_.find(bytes);
    Eigen::Vector3f norm = found == frtd_.frt_map_.end() ? fallback : found->second;
    if (!norm.allFinite() || norm.squaredNorm() < .5f) {
      PointVector neighbours; vector<float> distances;
      lidar_map_interface_->KNN(pt, 15, neighbours, distances);
      if (neighbours.size() >= 3) {
        Eigen::Vector3f center;
        computeNormalCell(neighbours, norm, center);
        if (norm.allFinite()) frtd_.frt_map_[bytes] = norm;
      }
    }
    // Target selection must not label excluded surfaces as observed. Native
    // DENSE/SPARSE transitions also define neighbours of wall frontiers.
    const bool floor_range=isFloorRangeFrontier(pt,norm,get_state(pt));
    if(pt.z<frtp_.target_min_z_ || pt.z>frtp_.target_max_z_ ||
        (!floor_range && frtp_.target_max_abs_normal_z_<1.f &&
         (!norm.allFinite() || std::abs(norm.z())>frtp_.target_max_abs_normal_z_)))return;
    frts2cluster.push_back(pt); frts_norm.push_back(norm);
  };
  for (size_t i=0;i<unclustered_frts_.size();++i)
    enqueue(unclustered_frts_[i], unclustered_norms_[i]);
  unclustered_frts_.clear(); unclustered_norms_.clear();

  if (cluster_list_.size() != 0) {
    cluster_list_.remove_if([this, &enqueue,
                             &cluster_removed](ClusterInfo::Ptr &cluster) {
      // if (!cluster->is_reachable_)
      //   return false;
      if (force_recluster_.count(cluster->id_) ||
          (force_recluster_.empty() &&
           has_overlap(cluster->box_max_, cluster->box_min_))) {
        for (auto &pt : cluster->cells_) {
          if (get_state(pt) == FRONTIER_DIS || get_state(pt) == FRONTIER_DIR) {
            enqueue(pt, Eigen::Vector3f::Zero());
          }
        }
        cluster_removed.push_back(cluster->id_);
        return true;
      }
      return false;
    });
  }

  for (const auto &pt : frt_new) enqueue(pt, Eigen::Vector3f::Zero());

  // 重新聚类:
  if (frts2cluster.size() < frtp_.cluster_minmum_point_num_) {
    unclustered_frts_ = frts2cluster; unclustered_norms_ = frts_norm;
    unclustered_frts_.insert(unclustered_frts_.end(),unsupported.begin(),unsupported.end());
    unclustered_norms_.insert(unclustered_norms_.end(),unsupported_norms.begin(),unsupported_norms.end());
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr frt_pc(
      new pcl::PointCloud<pcl::PointXYZ>);
  frt_pc->points = frts2cluster;
  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(frt_pc);
  auto getNbrs = [&](int norm_idx, int idx, vector<int> &nbr_idxs) -> int {
    nbr_idxs.clear();
    std::vector<int> indices;
    std::vector<float> squared_distances;
    if (kdtree.radiusSearch(idx, frtp_.cluster_min_radius_, indices,
                            squared_distances) < 3)
      return 0;
    Eigen::Vector3f norm = frts_norm[norm_idx];
    for (int nbr_idx : indices) {
      Eigen::Vector3f nbr_norm = frts_norm[nbr_idx];
      if (norm.dot(nbr_norm) > (frtp_.cluster_direction_radius_)) {
        nbr_idxs.push_back(nbr_idx);
      }
    }
    return nbr_idxs.size();
  };
  std::vector<int> labels;
  labels.resize(frt_pc->points.size(), -1); // 初始化标签，-1 表示未访问
  int cluster_id = 0;                       // 聚类ID
  for (size_t i = 0; i < frts2cluster.size(); i++) {
    if (labels[i] != -1)
      continue;
    std::vector<int> indices;
    if (getNbrs(i, i, indices) < 1)
      continue;
    cluster_id++;           // 分配新的聚类ID
    labels[i] = cluster_id; // 标记当前点
    std::list<size_t> queue;
    queue.push_back(i);
    Eigen::Vector3f aabb_max =
        Eigen::Vector3f(std::numeric_limits<float>::lowest(),
                        std::numeric_limits<float>::lowest(),
                        std::numeric_limits<float>::lowest());
    Eigen::Vector3f aabb_min = Eigen::Vector3f(
        std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());

    while (!queue.empty()) {
      size_t current = queue.front();
      queue.pop_front();
      // 执行基于半径的搜索找到当前点的邻居
      if (getNbrs(i, current, indices) >= 1) {
        Eigen::Vector3f norm = frts_norm[current];
        for (size_t j = 0; j < indices.size(); ++j) {
          int neighbor_index = indices[j];
          if (labels[neighbor_index] == -1) {
            // 如果邻居未被访问，则将其添加到聚类中
            labels[neighbor_index] = cluster_id;
            queue.push_back(neighbor_index);
          } else if (labels[neighbor_index] == 0) {
            // 如果邻居是噪声点，则将其重新标记为当前聚类的一部分
            labels[neighbor_index] = cluster_id;
          }
          aabb_min =
              aabb_min.cwiseMin(frts2cluster[neighbor_index].getVector3fMap());
          aabb_max =
              aabb_max.cwiseMax(frts2cluster[neighbor_index].getVector3fMap());
          if ((aabb_max - aabb_min).maxCoeff() > frtp_.cluster_radius_)
            break;
        }
      } else {
        // 如果邻域内的点数不足以形成一个聚类，则将其标记为噪声
        labels[i] = 0;
      }
    }
  }
  for (int i = 1; i <= cluster_id; i++) {
    PointVector frt_cluster_pt;
    vector<Eigen::Vector3f> frt_cluster_norm;
    for (int j = 0; j < labels.size(); j++) {
      if (labels[j] == i) {
        frt_cluster_pt.push_back(frts2cluster[j]);
        frt_cluster_norm.push_back(frts_norm[j]);
      }
    }
    // if (frt_cluster_pt.size() < frtp_.cluster_minmum_point_num_)
    //   continue;
    ClusterInfo::Ptr cluster = make_shared<ClusterInfo>();
    compute_cluster_info(frt_cluster_pt, frt_cluster_norm, cluster);
    cluster_list_.push_back(cluster);
    new_clusters.push_back(cluster);
  }
  // 将噪音删掉
  for (int i = 0; i < labels.size(); i++) {
    if (labels[i] < 0) {
      // Sparse arrivals were never assigned to a cluster. Keep them for the
      // next update; otherwise they remain visible frontiers forever but can
      // never reach viewpoint generation or TSP again.
      unclustered_frts_.push_back(frts2cluster[i]);
      unclustered_norms_.push_back(frts_norm[i]);
    } else if (labels[i] == 0) {
      ByteArrayRaw bytes;
      pos2bytes(frts2cluster[i], bytes);
      frtd_.label_map_[bytes] = DENSE;
      frtd_.frt_map_.erase(bytes);
    }
  }
  // Recheck unsupported labels against every new measured map. They are not
  // actionable completion blockers, but can become genuine surface targets.
  unclustered_frts_.insert(unclustered_frts_.end(),unsupported.begin(),unsupported.end());
  unclustered_norms_.insert(unclustered_norms_.end(),unsupported_norms.begin(),unsupported_norms.end());
}

void FrontierManager::idx2pos(const Eigen::Vector3i &idx, PointType &pt) {
  Eigen::Vector3f pt_v3f =
      (idx.cast<float>() + 0.5 * Eigen::Vector3f::Ones()) * frtp_.cell_size_ +
      frtp_.map_min_;
  pt.x = pt_v3f.x();
  pt.y = pt_v3f.y();
  pt.z = pt_v3f.z();
}

void FrontierManager::computeNormal(const PointVector &local_pts,
                                    Eigen::Vector3f &normal) {
  if (local_pts.size() < 3) {
    ROS_ERROR("computeNormal input size < 3");
    exit(1);
  }
  Eigen::Vector3f center(0.0, 0.0, 0.0);
  for (int i = 0; i < local_pts.size(); i++) {
    center += local_pts[i].getVector3fMap();
  }
  center /= local_pts.size();
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();

  for (auto &pt : local_pts) {
    Eigen::Vector3f div = pt.getVector3fMap() - center;
    covariance += div * div.transpose();
  }
  covariance /= local_pts.size();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> saes(covariance);
  normal = saes.eigenvectors().col(0);
  normal.normalize();
}

void FrontierManager::computeNormalCell(const PointVector &local_pts,
                                        Eigen::Vector3f &normal,
                                        Eigen::Vector3f &center) {
  if (local_pts.size() < 3) {
    ROS_ERROR("computeNormal input size < 3");
    exit(1);
  }
  center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  for (int i = 0; i < local_pts.size(); i++) {
    center += local_pts[i].getVector3fMap();
  }
  center /= local_pts.size();
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();

  for (auto &pt : local_pts) {
    Eigen::Vector3f div = pt.getVector3fMap() - center;
    covariance += div * div.transpose();
  }

  covariance /= local_pts.size();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> saes(covariance);
  normal = saes.eigenvectors().col(0);
  normal.normalize();
  Eigen::Vector3f dir =
      center - lidar_map_interface_->ld_->lidar_pose_.cast<float>();
  if (dir.dot(normal) < 0) {
    return;
  } else {
    normal = -normal;
  }

  Eigen::Vector3f move_pt_ =
      lidar_map_interface_->ld_->lidar_pose_.cast<float>() - center;
  double dotProduct = move_pt_.dot(normal);
  double normVec1 = move_pt_.norm();
  double normVec2 = normal.norm();
  double cosAngle = dotProduct / (normVec1 * normVec2);
  double angle = std::acos(cosAngle);
  angle = angle * 180.0 / M_PI;
  if (angle > 180)
    angle = 360 - angle;
  if (angle > 90)
    normal = -normal;
}

void FrontierManager::update_lidar_fov_edge(const vector<float> &depth) {
  auto viz_img = [&](cv::Mat img, string name) {
    cv::Mat image_8u, upsampled;
    cv::normalize(img, image_8u, 0, 255, cv::NORM_MINMAX);
    image_8u.convertTo(image_8u, CV_8UC1);
    cv::resize(image_8u, upsampled, cv::Size(400, 800));
    cv::imshow(name, upsampled);
    cv::waitKey(1);
  };
  frtd_.is_fov_edge_ = vector<bool>(20000, false);

  for (int j = 0; j < 200; j++) {
    for (int i = 0; i < 100; i++) {
      if (depth[i * 200 + j] <= 0.1)
        frtd_.is_fov_edge_[i * 200 + j] = true;
      else {
        frtd_.is_fov_edge_[i * 200 + j] = true;
        break;
      }
    }
    for (int i = 99; i >= 0; i--) {
      if (depth[i * 200 + j] <= 0.1)
        frtd_.is_fov_edge_[i * 200 + j] = true;
      else {
        frtd_.is_fov_edge_[i * 200 + j] = true;
        break;
      }
    }
  }
  cv::Mat img_origin(100, 200, CV_8UC1);
  for (int j = 0; j < 200; j++) {
    for (int i = 0; i < 100; i++) {
      img_origin.at<uchar>(i, j) = frtd_.is_fov_edge_[i * 200 + j];
    }
  }
  // viz_img(img_origin, "is_fov_edge");
}

bool FrontierManager::is_fov_edge(const PointType &pt) {
  return frtd_.is_fov_edge_[surface_pos2idx(pt)];
}

void FrontierManager::updateFrontierClusters(
    vector<ClusterInfo::Ptr> &cluster_updated, vector<int> &cluster_removed) {
  PointVector frt_new;
  bool trust_close_fov_edge;
  nh_.param("FrontierManager/trust_close_fov_edge",trust_close_fov_edge,false);
  auto has_dense_nbr = [&](const Eigen::Vector3i &idx) -> bool {
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          if (i == 0 && j == 0 && k == 0)
            continue;
          if (get_state(idx + Eigen::Vector3i(i, j, k)) == DENSE) {
            return true;
          }
        }
      }
    }
    return false;
  };
  auto has_sparse_nbr = [&](const Eigen::Vector3i &idx) -> bool {
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          if (i == 0 && j == 0 && k == 0)
            continue;
          if (get_state(idx + Eigen::Vector3i(i, j, k)) == SPARSE ||
              get_state(idx + Eigen::Vector3i(i, j, k)) == FRONTIER_DIR ||
              get_state(idx + Eigen::Vector3i(i, j, k)) == FRONTIER_DIS) {
            return true;
          }
        }
      }
    }
    return false;
  };

  epic_ros::Time t1 = epic_ros::Time::now();
  vector<Eigen::Vector3i> cells_2_update;
  update_lidar_pos();
  // Step1: 更新视角
  epic_ros::Time t2 = epic_ros::Time::now();
  static vector<PointVector> pts_vec;
  static int idx = 0;
  if (pts_vec.size() < 5) {
    pts_vec.push_back(lidar_map_interface_->ld_->lidar_cloud_.points);
    idx++;
  } else {
    idx = idx % 5;
    pts_vec[idx] = lidar_map_interface_->ld_->lidar_cloud_.points;
    idx++;
  }
  vector<float> depth = vector<float>(20000, -0.1);
  project_pts_2_depth_image(lidar_map_interface_->ld_->lidar_cloud_.points, depth);
  update_lidar_fov_edge(depth); // handle 雷达保护罩/旋翼/近点之类的东西
  for (int i = 0; i < pts_vec.size(); i++) {
    if (i == idx - 1)
      continue;
    project_pts_2_depth_image(pts_vec[i], depth);
  }
  update_lidar_pt_gap(depth);
  // 把gap-point可视化出来
  get_cells_2_update(lidar_map_interface_->ld_->lidar_cloud_.points,
                     cells_2_update);
  for (auto &cell : cells_2_update) {
    ByteArrayRaw bytes;
    idx2bytes(cell, bytes);
    frtd_.frt_map_.erase(bytes);
  }
  Eigen::Vector3f lidar_position =
      lidar_map_interface_->ld_->lidar_pose_.cast<float>();

  PointVector bad_observation, good_observation;
  for (auto &cell : cells_2_update) {
    PointType pt;
    idx2pos(cell, pt);
    if (is_gap_point(pt) || is_fov_edge(pt) ||
        (pt.getVector3fMap() - lidar_position).norm() >
            frtp_.good_observation_trust_length_) {
      bad_observation.push_back(pt);
    } else
      good_observation.push_back(pt);
  }
  viz_point(bad_observation, "bad_obs");
  viz_point(good_observation, "good_obs");
  // cout << "update and vizgap: " << (epic_ros::Time::now() - t2).toSec() * 1000
  // <<"----------------------------------------"<< endl;
  unordered_set<Eigen::Vector3i, Vector3i_Hash> old_frt_cells;
  old_frt_cells.reserve(cells_2_update.size());
  for (auto &cell : cells_2_update) {
    if (get_state(cell) == FRONTIER_DIS || get_state(cell) == FRONTIER_DIR)
      old_frt_cells.insert(cell);
  }
  // cout << "get_cells_2_update: " << (epic_ros::Time::now() - t1).toSec() * 1000 <<
  // "----------------------------------------"<< endl;

  /*
  距离合适 && 视角合理->good
  距离过长 || 视角过大->bad
  距离特别短 ->good
  */

  epic_ros::Time t3 = epic_ros::Time::now();

  // Step2: 分类，距离特别短的->good, 距离合适 && 视角合理->good
  vector<Eigen::Vector3i> cells_2_box_search;
  cells_2_box_search.reserve(cells_2_update.size());
  unordered_set<Eigen::Vector3i, Vector3i_Hash> bad_dis_set, bad_dir_set;
  for (int i = 0; i < cells_2_update.size(); i++) {
    ByteArrayRaw bytes;
    idx2bytes(cells_2_update[i], bytes);
    if (get_state(cells_2_update[i]) == DENSE) {
      continue;
    }
    PointType pt;
    idx2pos(cells_2_update[i], pt);
    float view_distance = (pt.getVector3fMap() - lidar_position).norm();
    if (view_distance < frtp_.good_observation_force_trust_length_ &&
        (trust_close_fov_edge || !is_fov_edge(pt))) {
      // if (view_distance < frtp_.good_observation_force_trust_length_) {
      frtd_.label_map_[bytes] = DENSE;
      continue;
    }
    bool bad_dir = is_gap_point(pt) || is_fov_edge(pt);
    bool bad_dis = view_distance > frtp_.good_observation_trust_length_;
    if (!bad_dir && !bad_dis) {
      frtd_.label_map_[bytes] = DENSE;
      continue;
    } else if (bad_dis) {
      bad_dis_set.insert(cells_2_update[i]);
    } else {
      bad_dir_set.insert(cells_2_update[i]);
    }
    cells_2_box_search.push_back(cells_2_update[i]);
  }
  // Step3: 分类，距离过长 || 视角过大->bad ,
  // 但要计算法向量判断一下是否是噪声点，如果是噪声点也设成good
  vector<PointVector> pts_inside;
  // cout << "prepare && split dense node: " << (epic_ros::Time::now() - t3).toSec()
  // * 1000 << endl;
  epic_ros::Time t4_ = epic_ros::Time::now();
  get_pts_in_cells(cells_2_box_search, pts_inside);
  // cout << "get_pts_in_cells " << (epic_ros::Time::now() - t4_).toSec() * 1000 <<
  // endl;
  epic_ros::Time t4 = epic_ros::Time::now();

  // 全都设置成sparse先
  unordered_set<Eigen::Vector3i, Vector3i_Hash> old_frt_set;
  for (auto &cell : cells_2_box_search) {
    ByteArrayRaw bytes;
    idx2bytes(cell, bytes);
    if (get_state(cell) == FRONTIER_DIR || get_state(cell) == FRONTIER_DIS) {
      old_frt_set.insert(cell);
    }
    frtd_.label_map_[bytes] = SPARSE;
    frtd_.frt_map_[bytes] = Eigen::Vector3f::Zero();
  }

  omp_set_num_threads(4);
  // clang-format off
  // Map insertion and neighbour labels share mutable state; serialize commits.
  // clang-format on
  for (int i = 0; i < cells_2_box_search.size(); i++) {
    auto local_pts = pts_inside[i];
    if (local_pts.size() < 3)
      continue;
    else {
      ByteArrayRaw bytes;
      idx2bytes(cells_2_box_search[i], bytes);
      Eigen::Vector3f norm;
      Eigen::Vector3f t;
      // 使用view_directon(类似ray-cast)去噪
      computeNormalCell(local_pts, norm, t);

      Eigen::Vector3i norm_nbr_1, norm_nbr_2, norm_nbr_3;
      PointType pt;
      idx2pos(cells_2_box_search[i], pt);
      // norm = (lidar_position - pt.getVector3fMap()).normalized();
      pos2idx(pt.getVector3fMap() + norm * frtp_.cell_size_, norm_nbr_1);
      pos2idx(pt.getVector3fMap() - norm * frtp_.cell_size_, norm_nbr_2);
      pos2idx(pt.getVector3fMap() - 2 * norm * frtp_.cell_size_, norm_nbr_3);
      if (get_state(norm_nbr_1) == DENSE || get_state(norm_nbr_2) == DENSE ||
          get_state(norm_nbr_3) == DENSE) {
        frtd_.label_map_[bytes] = DENSE; // 说明这个点是噪声点
        // frt_map_[cells_2_box_search[i]] = norm;

        continue;
      } else {
        frtd_.frt_map_[bytes] = norm;
      }
    }
  }
  frt_new.clear();
  frt_new.reserve(cells_2_box_search.size());
  for (auto &cell : cells_2_box_search) {
    ByteArrayRaw bytes;
    idx2bytes(cell, bytes);
    PointType range_point;idx2pos(cell,range_point);
    const auto normal=frtd_.frt_map_.find(bytes);
    // A distance frontier on measured ground need not touch a dense cell:
    // oblique LiDAR beams can leave a gap exactly at the trusted range.
    const CELL_STATE floor_state=bad_dis_set.count(cell)?FRONTIER_DIS:FRONTIER_DIR;
    const bool floor_range_boundary=(bad_dis_set.count(cell) || bad_dir_set.count(cell)) && normal!=frtd_.frt_map_.end() &&
        isFloorRangeFrontier(range_point,normal->second,floor_state) &&
        (range_point.getVector3fMap()-lidar_position).norm()<=
            frtp_.good_observation_trust_length_+2.f*frtp_.cell_size_;
    if (get_state(cell) == SPARSE && (has_dense_nbr(cell) || floor_range_boundary)) {
      if (bad_dis_set.count(cell)) {
        frtd_.label_map_[bytes] = FRONTIER_DIS;
      } else if (bad_dir_set.count(cell)) {
        frtd_.label_map_[bytes] = FRONTIER_DIR;
      } else {
        ROS_ERROR("wtf 885");
        exit(1);
      }
      if (old_frt_set.count(cell) == 0) {
        PointType pt;
        idx2pos(cell, pt);
        frt_new.push_back(pt);
      }
    } else {
      frtd_.frt_map_.erase(bytes);
    }
  }

  // cout << "set normal " << (epic_ros::Time::now() - t4).toSec() * 1000 << endl;
  epic_ros::Time t5 = epic_ros::Time::now();
  PointVector updated_frt_pts;
  for (auto &cell : old_frt_cells) {
    if (get_state(cell) != FRONTIER_DIR && get_state(cell) != FRONTIER_DIS) {
      // updated_frt_pts.emplace_back
      PointType pt;
      idx2pos(cell, pt);
      updated_frt_pts.emplace_back(pt);
    }
  }
  PointVector updated_pts;
  updated_pts.insert(updated_pts.end(), updated_frt_pts.begin(),
                     updated_frt_pts.end());
  updated_pts.insert(updated_pts.end(), frt_new.begin(), frt_new.end());
  update_updating_aabb(updated_pts);
  cluster_frts(frt_new, cluster_updated, cluster_removed);
}

int FrontierManager::surface_pos2idx(const PointType &pt) {
  Eigen::Vector3f pt_lidar_frame = transform_world2lidar * pt.getVector3fMap();
  Eigen::Vector2i surface_idx;
  Sphere_PosToIndex(Eigen::Vector3f::Zero(), pt_lidar_frame,
                                      surface_idx);
  return surface_idx.x() * 200 + surface_idx.y();
}

void FrontierManager::update_lidar_pos() {
  transform_world2lidar = Eigen::Isometry3f::Identity();
  transform_world2lidar.translate(
      lidar_map_interface_->ld_->lidar_pose_.cast<float>());
  transform_world2lidar.rotate(lidar_map_interface_->ld_->lidar_q_.cast<float>());
  transform_world2lidar = transform_world2lidar.inverse();
}

void FrontierManager::project_pts_2_depth_image(PointVector &pts_vec,
                                                vector<float> &depth_img) {
  PointVector pts_lidar_frame;
  // depth_img = vector<float>(20000, -0.1);
  auto project_pt = [&](PointType &pt) {
    Eigen::Vector3f pt_lidar_frame =
        transform_world2lidar * pt.getVector3fMap();
    float dis = pt_lidar_frame.norm();
    if (dis > frtp_.update_length_)
      return;
    Eigen::Vector2i surface_idx;
    Sphere_PosToIndex(Eigen::Vector3f::Zero(), pt_lidar_frame,
                                        surface_idx);
    if (depth_img[surface_idx.x() * 200 + surface_idx.y()] < 0 ||
        depth_img[surface_idx.x() * 200 + surface_idx.y()] > dis) {
      depth_img[surface_idx.x() * 200 + surface_idx.y()] = dis;
    }
  };
  for (auto &pt : pts_vec)
    project_pt(pt);
}

void FrontierManager::update_updating_aabb(const PointVector &new_frt_pts) {
  frtd_.updating_aabb_min = Eigen::Vector3f(std::numeric_limits<float>::max(),
                                            std::numeric_limits<float>::max(),
                                            std::numeric_limits<float>::max());
  frtd_.updating_aabb_max =
      Eigen::Vector3f(std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::lowest());
  for (auto &p : new_frt_pts) {
    frtd_.updating_aabb_min =
        frtd_.updating_aabb_min.cwiseMin(p.getVector3fMap());
    frtd_.updating_aabb_max =
        frtd_.updating_aabb_max.cwiseMax(p.getVector3fMap());
  }
  frtd_.updating_aabb_min -= Eigen::Vector3f::Ones() * 0.1;
  frtd_.updating_aabb_max += Eigen::Vector3f::Ones() * 0.1;
}

void FrontierManager::compute_cluster_info(
    const PointVector &frt_pts, const vector<Eigen::Vector3f> &frt_norms,
    ClusterInfo::Ptr cluster) {
  static int id = 0;
  cluster->center_.setZero();
  cluster->normal_.setZero();
  cluster->cells_.resize(frt_pts.size());
  cluster->norms_.resize(frt_pts.size());
  cluster->box_max_ = Eigen::Vector3f(std::numeric_limits<float>::lowest(),
                                      std::numeric_limits<float>::lowest(),
                                      std::numeric_limits<float>::lowest());
  cluster->box_min_ = Eigen::Vector3f(std::numeric_limits<float>::max(),
                                      std::numeric_limits<float>::max(),
                                      std::numeric_limits<float>::max());
  for (int i = 0; i < frt_pts.size(); i++) {
    Eigen::Vector3f pt = frt_pts[i].getVector3fMap();
    Eigen::Vector3f norm = frt_norms[i];
    cluster->center_ += pt;
    cluster->normal_ += norm;
    cluster->box_max_ = cluster->box_max_.cwiseMax(pt);
    cluster->box_min_ = cluster->box_min_.cwiseMin(pt);
    cluster->cells_[i] = PointType(pt.x(), pt.y(), pt.z());
    cluster->norms_[i] = norm;
  }
  cluster->box_max_ += Eigen::Vector3f::Ones() * 0.1;
  cluster->box_min_ -= Eigen::Vector3f::Ones() * 0.1;
  cluster->center_ /= (float)frt_pts.size();
  cluster->normal_.normalize();
  cluster->id_ = id++;
  cluster->is_dormant_ = false;
  // cluster->is_reachable_ = false;
  cluster->is_reachable_ = true;
  if ((cluster->box_max_ - cluster->box_min_).maxCoeff() <
      frtp_.cluster_min_size_)
    cluster->is_dormant_ = true;
  if (cluster->cells_.size() < frtp_.cluster_minmum_point_num_)
    cluster->is_dormant_ = true;
  cluster->is_new_cluster_ = true;
}

bool FrontierManager::has_overlap(const Eigen::Vector3f &box_max_,
                                  const Eigen::Vector3f &box_min_) {
  if (frtd_.updating_aabb_max.x() < box_min_.x() ||
      frtd_.updating_aabb_max.y() < box_min_.y() ||
      frtd_.updating_aabb_max.z() < box_min_.z() ||
      frtd_.updating_aabb_min.x() > box_max_.x() ||
      frtd_.updating_aabb_min.y() > box_max_.y() ||
      frtd_.updating_aabb_min.z() > box_max_.z()) {
    return false;
  }
  return true;
}

void FrontierManager::updateHalfSpaces(vector<ClusterInfo::Ptr> &clusters) {
  auto getNbrs = [&](Eigen::Vector3i &idx, vector<Eigen::Vector3i> &nbrs) {
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          if (i == 0 && j == 0 && k == 0)
            continue;
          nbrs.emplace_back(idx[0] + i, idx[1] + j, idx[2] + k);
        }
      }
    }
  };
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &cluster : clusters) {
    unordered_set<Eigen::Vector3i, Vector3i_Hash> dense, sparse;
    for (auto &cell : cluster->cells_) {
      Eigen::Vector3i idx;
      pos2idx(cell, idx);
      vector<Eigen::Vector3i> nbrs;
      getNbrs(idx, nbrs);
      for (auto &nbr : nbrs) {
        if (get_state(nbr) == DENSE) {
          dense.insert(nbr);
        } else if (get_state(nbr) == UNKNOWN) {
          continue;
        } else {
          sparse.insert(nbr);
        }
      }
    }
    Eigen::Vector3f sparse_center = Eigen::Vector3f::Zero();
    for (auto &cell : sparse) {
      PointType pt;
      idx2pos(cell, pt);
      sparse_center += pt.getVector3fMap();
    }
    sparse_center /= sparse.size();
    Eigen::Vector3f dense_center = Eigen::Vector3f::Zero();
    for (auto &cell : dense) {
      PointType pt;
      idx2pos(cell, pt);
      sparse_center += pt.getVector3fMap();
    }
    dense_center /= dense.size();
    Eigen::Vector3f dir = sparse_center - dense_center;
    dir.z() = 0;
    dir.normalize();
    cluster->view_halfspace_ =
        Eigen::Vector4f(dir.x(), dir.y(), dir.z(), -cluster->center_.dot(dir));
  }
}

inline bool FrontierManager::isInBox(const PointType &pt) {
  return lidar_map_interface_->IsInBox(pt);
}

inline bool FrontierManager::isInBox(const Eigen::Vector3f &pt) {
  return lidar_map_interface_->IsInBox(pt);
}

void FrontierManager::selectBestViewpoint(ClusterInfo::Ptr &cluster) {
  if (cluster->vp_clusters_.empty()) {
    return;
  }
  PointVector vps;
  for (auto &vp_cluster : cluster->vp_clusters_) {
    vps.insert(vps.end(), vp_cluster.vps_.begin(), vp_cluster.vps_.end());
  }
  vector<float> score(vps.size(), 0);
  vector<float> yaw(vps.size(), 0);
  vector<PointVector> occ_free_frts; // raycast成功，但没有考虑视角
  occ_free_frts.resize(vps.size(), PointVector());
  RayCaster ray_caster;
  ray_caster.setParams(double(frtp_.cell_size_), frtp_.map_min_.cast<double>());
  const double now=epic_ros::Time::now().toSec();
  size_t deferred_count=0;
  bool measured_surface=false;
  nh_.param("FrontierManager/measured_surface_visibility",measured_surface,false);
  vector<Eigen::Vector3f> target_surface(cluster->cells_.size(),Eigen::Vector3f::Zero());
  vector<bool> target_supported(cluster->cells_.size(),false);
  if(measured_surface)for(size_t j=0;j<cluster->cells_.size();++j) {
    PointVector nearest;vector<float> distances;
    lidar_map_interface_->KNN(cluster->cells_[j],1,nearest,distances);
    if(!nearest.empty()){target_surface[j]=nearest[0].getVector3fMap();target_supported[j]=true;}
  }
  unordered_map<Eigen::Vector3i,Eigen::Vector3f,Vector3i_Hash> blocker_surface;
  size_t self_surface_skipped=0,backface_reject=0;
  size_t range_reject=0,state_range_reject=0,direction_reject=0,occluded=0;
  double first_block_support=-1.,first_block_from_target=-1.;
  for (int i = 0; i < vps.size(); i++) {
    Eigen::Vector3f vp = vps[i].getVector3fMap();
    bool deferred=false;
    for(const auto &entry:deferred_viewpoints_)
      if(entry.second>now && (vp-entry.first).norm()<1.f){deferred=true;break;}
    if(deferred){++deferred_count;continue;}
    for (int j = 0; j < cluster->cells_.size(); j++) {
      Eigen::Vector3f frt = cluster->cells_[j].getVector3fMap();
      Eigen::Vector3f dir = (frt - vp).cast<float>();
      float distance = dir.norm();
      dir.normalize();
      if (distance > frtp_.good_observation_trust_length_) {++range_reject;continue;}
      CELL_STATE state = get_state(cluster->cells_[j]);
      if (state == FRONTIER_DIR &&
          distance > frtp_.good_observation_force_trust_length_) {++state_range_reject;continue;}
      Eigen::Vector3f norm = cluster->norms_[j];
      float sin_theta = dir.dot(norm);
      if(measured_surface) {
        // Normals face the measuring robot. frt-vp has the opposite sign.
        sin_theta=-sin_theta;
        if(sin_theta<=0.f){++backface_reject;continue;}
      }
      float cos_thera = sqrt(std::max(0.f,1 - sin_theta * sin_theta));
      float delta = M_PI / 100.0;
      float score = sin_theta / (sin_theta + delta * cos_thera);
      if (score < frtp_.good_observation_direction_score_) {++direction_reject;continue;}
      Eigen::Vector3f ray_start=frt;
      if(frtp_.floor_range_frontiers_ && norm.z()>.8f)
        ray_start+=norm*(.6f*frtp_.cell_size_); // Skip only the target floor's own surface voxel.
      ray_caster.input(ray_start.cast<double>(), vp.cast<double>());
      bool visib = true;
      Eigen::Vector3i idx;
      while (ray_caster.nextId(idx)) {
        // 必须在box里
        CELL_STATE state = get_state(idx);
        PointType pt;
        idx2pos(idx, pt);
        const bool surface_label=state==DENSE || state==SPARSE;
        if(surface_label && measured_surface && target_supported[j] &&
            (vp-target_surface[j]).dot(norm)>0.f &&
            (pt.getVector3fMap()-frt).norm()<=1.75f*frtp_.cell_size_) {
          auto found=blocker_surface.find(idx);
          if(found==blocker_surface.end()) {
            PointVector nearest;vector<float> distances;
            lidar_map_interface_->KNN(pt,1,nearest,distances);
            Eigen::Vector3f support=Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
            if(!nearest.empty())support=nearest[0].getVector3fMap();
            found=blocker_surface.emplace(idx,support).first;
          }
          // A quantized target can sit behind an adjacent DENSE voxel of the
          // SAME measured plane. Ignore only that local self-occlusion; an
          // intervening surface with a different plane still blocks the ray.
          if(found->second.allFinite() &&
              std::abs((found->second-target_surface[j]).dot(norm))<=.25f*frtp_.cell_size_) {
            ++self_surface_skipped;continue;
          }
        }
        if (!lidar_map_interface_->IsInBox(pt) || surface_label) {
          ++occluded;
          if(first_block_support<0.) {
            first_block_support=lidar_map_interface_->getDisToOcc(pt);
            first_block_from_target=(pt.getVector3fMap()-frt).norm();
          }
          visib = false;
          break;
        }
      }
      if (visib) {
        occ_free_frts[i].push_back(cluster->cells_[j]);
      }
    }
  }
  for (int i = 0; i < vps.size(); i++) {
    if (occ_free_frts[i].size() < 3) {
      continue;
    }
    Eigen::Vector3f vp = vps[i].getVector3fMap();
    vector<int> yaw_score = vector<int>(8, 0);
    for (int j = -4; j < 4; j++) {
      float yaw = (45.0 * j + 22.5) * M_PI / 180.0;
      if (yaw > M_PI)
        yaw -= 2 * M_PI;
      if (yaw < -M_PI)
        yaw += 2 * M_PI;
      Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
      transform.rotate(Eigen::AngleAxisf(-vpp_.lidar_pitch_ * M_PI / 180.0,
                                         Eigen::Vector3f::UnitY()));
      transform.rotate(Eigen::AngleAxisf(-yaw, Eigen::Vector3f::UnitZ()));
      for (auto &pt : occ_free_frts[i]) {
        Eigen::Vector3f pt2see = transform * (pt.getVector3fMap() - vp);
        float pitch = atan2(pt2see.z(), sqrt(pt2see.x() * pt2see.x() +
                                             pt2see.y() * pt2see.y()));
        if (pitch > vpp_.fov_up_ || pitch < vpp_.fov_down_)
          continue;
        yaw_score[j + 4]++;
      }
    }
    int max_yaw_idx = distance(yaw_score.begin(),
                               max_element(yaw_score.begin(), yaw_score.end()));
    // if (yaw_score[max_yaw_idx] == 0)
    //   continue;
    score[i] = yaw_score[max_yaw_idx];
    // Eigen::Vector4f hs = cluster->view_halfspace_;
    // Eigen::Vector4f vp_h(vp.x(), vp.y(), vp.z(), 1.0);
    // if (vp_h.dot(hs) < -0.1) {
    //   score[i] *= (1.5 - vp_h.dot(hs));
    // }
    yaw[i] = (45.0 * (max_yaw_idx - 4) + 22.5) / 180.0 * M_PI;
  }
  if(frtp_.floor_range_frontiers_ && cluster->normal_.z()>.8f) {
    Eigen::Vector3f forward=cluster->center_-lidar_map_interface_->ld_->lidar_pose_.cast<float>();
    forward.z()=0.f;
    bool beyond=false;
    for(size_t i=0;i<vps.size();++i)if(score[i]>0 &&
        (vps[i].getVector3fMap()-cluster->center_).dot(forward)>=0.f)beyond=true;
    if(beyond)for(size_t i=0;i<vps.size();++i)
      if((vps[i].getVector3fMap()-cluster->center_).dot(forward)<0.f)score[i]=0.f;
  }
  int best_vp_idx = std::distance(score.begin(),
                                  std::max_element(score.begin(), score.end()));
  if(debug_viewpoints_) {
    size_t visible=0;for(const auto &points:occ_free_frts)visible+=points.size();
    ROS_INFO("VP_SCORE id=%d center=(%.2f,%.2f,%.2f) normal_z=%.2f samples=%zu deferred=%zu visible_pairs=%zu scored=%zu best=%.0f support=%.3f range_reject=%zu state_range_reject=%zu direction_reject=%zu occluded=%zu first_block_support=%.3f first_block_from_target=%.3f measured_surface=%d self_surface_skipped=%zu backface_reject=%zu",
        cluster->id_,cluster->center_.x(),cluster->center_.y(),cluster->center_.z(),cluster->normal_.z(),
        vps.size(),deferred_count,visible,size_t(std::count_if(score.begin(),score.end(),[](float s){return s>0.f;})),
        score[best_vp_idx],lidar_map_interface_->getDisToOcc(cluster->center_),range_reject,state_range_reject,
        direction_reject,occluded,first_block_support,first_block_from_target,
        int(measured_surface),self_surface_skipped,backface_reject);
  }
  if (score[best_vp_idx] == 0) {
    cluster->is_reachable_ = false;
    cluster->vp_clusters_.clear();
  } else {
    cluster->is_reachable_ = true;
    cluster->best_vp_yaw_ = yaw[best_vp_idx];
    cluster->best_vp_ = vps[best_vp_idx].getVector3fMap();
    const float yaw_error = std::atan2(std::sin(cluster->best_vp_yaw_ - graph_->odom_node_->yaw_),
                                       std::cos(cluster->best_vp_yaw_ - graph_->odom_node_->yaw_));
    if (((cluster->best_vp_ - graph_->odom_node_->center_).norm() < vpp_.reached_position_tolerance_) &&
        (std::abs(yaw_error) < vpp_.reached_yaw_tolerance_)) {
      cluster->is_reachable_ = false;
      cluster->is_dormant_ = true;
      cluster->vp_clusters_.clear();
    }
    int tmp_idx = best_vp_idx;
    for (auto &vpc : cluster->vp_clusters_) {
      if (tmp_idx < vpc.vps_.size()) {
        cluster->distance_ = vpc.distance_;
        break;
      } else {
        tmp_idx -= vpc.vps_.size();
      }
    }
  }
}

void FrontierManager::initClusterViewpoints(ClusterInfo::Ptr &cluster) {
  cluster->last_viewpoint_attempt_ = epic_ros::Time::now().toSec();
  cluster->vp_clusters_.clear();
  PointVector vps_init;
  vps_init.reserve(origin_viewpoints_.size());
  size_t occupied=0,outside=0,no_region=0;
  for (auto &ovp : origin_viewpoints_) {
    Eigen::Vector3f vp = ovp + cluster->center_;
    if(frtp_.floor_range_frontiers_ && cluster->normal_.z()>.8f &&
        cluster->center_.z()<lidar_map_interface_->ld_->lidar_pose_.z()+.5f)
      vp.z()=cluster->center_.z()+frtp_.floor_viewpoint_clearance_;
    else if (vpp_.fixed_height_enabled_) vp.z() = vpp_.fixed_height_;
    // Nearby walls retain cruise height; another observed level keeps native
    // 3D viewpoint sampling so it can create goals through a stair opening.
    else if(frtp_.wall_viewpoints_at_current_height_ &&
        std::abs(cluster->center_.z()-lidar_map_interface_->ld_->lidar_pose_.z())<2.4f)
      vp.z()=lidar_map_interface_->ld_->lidar_pose_.z();
    if (lidar_map_interface_->getDisToOcc(vp) < 0.9) {++occupied;continue;}
    if (!isInBox(vp)) {++outside;continue;}
    Eigen::Vector3i idx;
    graph_->getIndex(vp, idx);
    if (graph_->getRegionNode(idx) == nullptr) {++no_region;continue;}
    vps_init.emplace_back(vp.x(), vp.y(), vp.z());
  }
  if(debug_viewpoints_)ROS_INFO("VP_SAMPLE id=%d center=(%.2f,%.2f,%.2f) normal_z=%.2f raw=%zu occupied=%zu outside=%zu no_region=%zu accepted=%zu support=%.3f",
      cluster->id_,cluster->center_.x(),cluster->center_.y(),cluster->center_.z(),cluster->normal_.z(),
      origin_viewpoints_.size(),occupied,outside,no_region,vps_init.size(),lidar_map_interface_->getDisToOcc(cluster->center_));
  if (vps_init.empty()) {
    cluster->is_reachable_ = false;
    return;
  }
  pcl::PointCloud<PointType>::Ptr vp_cloud(new pcl::PointCloud<PointType>);
  vp_cloud->points = vps_init;
  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(vp_cloud);
  vector<float> radius_vec;
  radius_vec.resize(vps_init.size(), 0.0);
  for (int i = 0; i < vps_init.size(); i++) {
    radius_vec[i] = lidar_map_interface_->getDisToOcc(vps_init[i]);
  }
  // DB-SCAN 基于连通性将初始viewpoint聚成几类
  std::vector<int> labels;
  labels.resize(vps_init.size(), -1); // 初始化标签，-1 表示未访问
  auto getNbrs = [&](int idx, vector<int> &nbr_idx) -> int {
    vector<float> sqr_distances;
    PointType p = vps_init[idx];
    vector<int> nbrs_tmp;
    kdtree.radiusSearch(p, radius_vec[idx], nbrs_tmp, sqr_distances);
    nbr_idx.clear();
    for (int i = 0; i < nbrs_tmp.size(); i++) {
      if (labels[nbrs_tmp[i]] == -1)
        nbr_idx.push_back(nbrs_tmp[i]);
    }
    return nbr_idx.size();
  };

  int cluster_id = 0; // 聚类ID
  for (int i = 0; i < vps_init.size(); i++) {
    if (labels[i] != -1)
      continue;
    vector<int> nbr_idx;
    if (getNbrs(i, nbr_idx) == 0)
      continue;
    cluster_id++;
    labels[i] = cluster_id;
    std::list<size_t> queue;
    queue.push_back(i);
    while (!queue.empty()) {
      size_t current = queue.front();
      queue.pop_front();
      if (getNbrs(current, nbr_idx) == 0)
        continue;
      for (int j = 0; j < nbr_idx.size(); j++) {
        auto nbr = nbr_idx[j];
        if (labels[nbr] == -1) {
          labels[nbr] = cluster_id;
          queue.push_back(nbr);
        }
      }
    }
  }
  for (int i = 0; i < cluster_id; i++) {
    ViewpointCluster vp_cluster;
    vp_cluster.vps_.clear();
    vector<float> cls_radius_vec;
    vector<int> cls_idx_vec;
    for (int j = 0; j < labels.size(); j++) {
      if (labels[j] != i + 1)
        continue;
      vp_cluster.vps_.push_back(vps_init[j]);
      cls_radius_vec.push_back(radius_vec[j]);
    }
    for (int j = 0; j < cls_radius_vec.size(); j++) {
      cls_idx_vec.push_back(j);
    }
    sort(cls_idx_vec.begin(), cls_idx_vec.end(),
         [&](int a, int b) { return cls_radius_vec[a] > cls_radius_vec[b]; });
    vp_cluster.center_ = vp_cluster.vps_[cls_idx_vec[0]].getVector3fMap();
    for (int j = 0; j < cls_radius_vec.size(); j++) {
      Eigen::Vector3f pt(vp_cluster.vps_[cls_idx_vec[j]].x,
                         vp_cluster.vps_[cls_idx_vec[j]].y,
                         vp_cluster.vps_[cls_idx_vec[j]].z);
      Eigen::Vector3i idx;
      graph_->getIndex(pt, idx);
      auto region = graph_->getRegionNode(idx);
      if (region && !region->topo_nodes_.empty()) {
        vp_cluster.center_ = vp_cluster.vps_[cls_idx_vec[j]].getVector3fMap();
        break;
      }
    }
    cluster->vp_clusters_.push_back(vp_cluster);
  }
  std::sort(cluster->vp_clusters_.begin(), cluster->vp_clusters_.end(),
            [](const ViewpointCluster &a, const ViewpointCluster &b) {
              return a.vps_.size() > b.vps_.size();
            });
  // cluster->vp_clusters_.resize(min(16, int(cluster->vp_clusters_.size())));
}

void FrontierManager::removeUnreachableViewpoints(
    vector<ClusterInfo::Ptr> &clusters) {
  if (graph_->odom_node_->neighbors_.empty())
    return;
  // 建立一张映射表，可以通过topo-node映射到要删除的vp_cluster
  vector<int> nodeidx2clusteridx;
  vector<int> nodeidx2vpclusteridx;
  vector<TopoNode::Ptr> nodes2insert;
  for (int i = 0; i < clusters.size(); i++) {
    for (int j = 0; j < clusters[i]->vp_clusters_.size(); j++) {
      nodeidx2clusteridx.push_back(i);
      nodeidx2vpclusteridx.push_back(j);
      TopoNode::Ptr vp_node = make_shared<TopoNode>();
      vp_node->center_ = clusters[i]->vp_clusters_[j].center_;
      nodes2insert.push_back(vp_node);
    }
  }

  epic_ros::Time t1 = epic_ros::Time::now();
  graph_->insertNodes(nodes2insert,true);
  epic_ros::Time t2 = epic_ros::Time::now();
  vector<bool> vp_cluster_kept;
  vp_cluster_kept.resize(nodes2insert.size(), true);
  vector<size_t> no_connection(clusters.size(),0),search_failed(clusters.size(),0);
  // 可以并行
  for (int i = 0; i < nodes2insert.size(); i++) {
    if (nodes2insert[i]->neighbors_.empty()) {
      ++no_connection[nodeidx2clusteridx[i]];
      vp_cluster_kept[i] = false;
      continue;
    }
    vector<TopoNode::Ptr> topo_path;
    auto closest_node = graph_->odom_node_;
    float closest_dis =
        (closest_node->center_ - nodes2insert[i]->center_).squaredNorm();
    for (auto &hodom : graph_->history_odom_nodes_) {
      if ((hodom->center_ - nodes2insert[i]->center_).squaredNorm() <
          closest_dis) {
        closest_dis = (hodom->center_ - nodes2insert[i]->center_).squaredNorm();
        closest_node = hodom;
      }
    }
    if (!graph_->graphSearch(closest_node,nodes2insert[i],topo_path,3e-4)) {
      ++search_failed[nodeidx2clusteridx[i]];
      vp_cluster_kept[i] = false;
    } else {
      clusters[nodeidx2clusteridx[i]]
          ->vp_clusters_[nodeidx2vpclusteridx[i]]
          .distance_ = graph_->getPathLength(topo_path);
    }
  }
  graph_->removeNodes(nodes2insert);
  vector<unordered_set<int>> kept_vp_cluster;
  kept_vp_cluster.resize(clusters.size(), unordered_set<int>());
  for (int i = 0; i < vp_cluster_kept.size(); i++) {
    if (!vp_cluster_kept[i])
      continue;
    kept_vp_cluster[nodeidx2clusteridx[i]].insert(nodeidx2vpclusteridx[i]);
  }
  for (int i = 0; i < clusters.size(); i++) {
    if(debug_viewpoints_)ROS_INFO("VP_TOPO id=%d groups=%zu connected=%zu no_connection=%zu graph_failed=%zu",
        clusters[i]->id_,clusters[i]->vp_clusters_.size(),kept_vp_cluster[i].size(),no_connection[i],search_failed[i]);
    vector<ViewpointCluster> tmp;
    tmp.swap(clusters[i]->vp_clusters_);
    for (int j = 0; j < tmp.size(); j++) {
      if (kept_vp_cluster[i].find(j) == kept_vp_cluster[i].end())
        continue;
      clusters[i]->vp_clusters_.push_back(tmp[j]);
    }
    if (clusters[i]->vp_clusters_.empty())
      clusters[i]->is_reachable_ = false;
    else {
      clusters[i]->is_reachable_ = true;
      sort(clusters[i]->vp_clusters_.begin(), clusters[i]->vp_clusters_.end(),
           [](const ViewpointCluster &a, const ViewpointCluster &b) {
             return a.distance_ < b.distance_;
           });
      float min_distance = clusters[i]->vp_clusters_[0].distance_;
      vector<ViewpointCluster> tmp2;
      for (int j = 0; j < min(8, int(clusters[i]->vp_clusters_.size())); j++) {
        if (clusters[i]->vp_clusters_[j].distance_ <= min_distance * 1.35 ||
            clusters[i]->vp_clusters_[j].distance_ <=
                min_distance + vpp_.sample_pillar_max_radius_)
          tmp2.push_back(clusters[i]->vp_clusters_[j]);
      }
      clusters[i]->vp_clusters_.swap(tmp2);
    }
  }
}

void FrontierManager::printMemoryCost() {
  int label_map_size = frtd_.label_map_.size();
  int frt_map_size = frtd_.frt_map_.size();
  cout << "label_map_size: "
       << "(" << to_string(frtp_.idx_byte_size_) << " + 2) * " << label_map_size
       << " = " << (float(label_map_size * (frtp_.idx_byte_size_ + 2)) / 1024.0)
       << "KB" << endl;
  static epic_ros::Publisher mem_pub =
      nh_.advertise<std_msgs::msg::Float32>("/mem_cost", 1);
  static epic_ros::Publisher mem_pub_2 =
      nh_.advertise<std_msgs::msg::Float32>("/mem_cost_2", 1);
  static epic_ros::Publisher mem_pub_3 =
      nh_.advertise<std_msgs::msg::Float32>("/mem_cost_3", 1);
  std_msgs::msg::Float32 msg, msg_2, msg_3;
  msg.data = (float(label_map_size * (frtp_.idx_byte_size_ + 2)) / 1024.0);
  mem_pub.publish(msg);
  // 4+2 -> 4+1+8
  msg_2.data =
      (float(label_map_size * (frtp_.idx_byte_size_ + 1 + 8)) / 1024.0);
  //
  msg_3.data = (float(label_map_size * (frtp_.idx_byte_size_ + 1 + 8) +
                      frtd_.label_map_.bucket_count() * sizeof(void *)) /
                1024.0);

  cout << "frt_map_size2 = " << msg_2.data << endl;
  cout << "frt_map_size3 = " << msg_3.data << endl;
  mem_pub_2.publish(msg_2);
  mem_pub_3.publish(msg_3);
}
inline void
FrontierManager::Sphere_PosToIndex(const Eigen::Vector3f &lidar_center,
                                   const Eigen::Vector3f &pos,
                                   Eigen::Vector2i &id) {
  // double dis = sqrt(pow((pos(0)-lidar_center(0)),2) +
  // pow((pos(1)-lidar_center(1)),2) + pow((pos(2)-lidar_center(2)),2));
  double dis = (pos - lidar_center).norm(); // pos是被按回去的，卡了最大范围。
  double phi_x = atan2((pos(1) - lidar_center(1)),
                       (pos(0) - lidar_center(0))); // 水平面，以x为极轴的转角
  if (phi_x < 0)
    phi_x = 2 * M_PI + phi_x;                              // 范围是0-2pi
  double theta_z = acos((pos(2) - lidar_center(2)) / dis); // 以z为极轴的转角
  if (theta_z < 0)
    theta_z = 2 * M_PI + theta_z;
  double sphere_r = 1 / M_PI;
  Eigen::Vector2d Arc_l;
  Arc_l(0) = sphere_r * theta_z;
  Arc_l(1) = sphere_r * phi_x;
  for (int i = 0; i < 2; ++i) {
    id(i) = floor(Arc_l(i) * 100);
  }
}

bool FrontierManager::isFloorRangeFrontier(const PointType &point,
    const Eigen::Vector3f &normal,CELL_STATE state) const {
  const float body_z=lidar_map_interface_->ld_->lidar_pose_.z();
  const bool rising_edge=frtp_.ascending_floor_frontiers_ && state==FRONTIER_DIR &&
      point.z>body_z-frtp_.floor_viewpoint_clearance_+.3f;
  return frtp_.floor_range_frontiers_ && (state==FRONTIER_DIS || rising_edge) && normal.allFinite() &&
      normal.z()>.8f && point.z<body_z+.5f;
}

bool FrontierManager::hasInspectionFrontiers() const {
  // These clusters already passed the surface height/normal policy. Raw
  // frontier labels also contain excluded floor/ceiling cells.
  for(const auto &cluster:cluster_list_)
    if(cluster && !cluster->is_dormant_)return true;
  return false;
}

void FrontierManager::deferViewpoint(const Eigen::Vector3f &position,double seconds) {
  if(!position.allFinite() || seconds<=0.)return;
  const double until=epic_ros::Time::now().toSec()+seconds;
  for(auto &entry:deferred_viewpoints_)if((entry.first-position).norm()<1.f) {
    entry.second=std::max(entry.second,until);return;
  }
  if(deferred_viewpoints_.size()>=64)deferred_viewpoints_.erase(deferred_viewpoints_.begin());
  deferred_viewpoints_.emplace_back(position,until);
  // This changes candidate scheduling only. Frontier observation labels and
  // dormant/completed flags are deliberately untouched.
}
