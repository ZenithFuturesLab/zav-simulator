/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-04-14 21:44:58
 * @LastEditTime: 2024-04-15 13:30:53
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#include <epic_ros2/planning_workers.hpp>
#include <frontier_manager/frontier_manager.h>
#include <pcl/filters/voxel_grid.h>
#include <bitset>
#include <chrono>
#include <iterator>
#include <opencv2/imgproc.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
size_t ByteArrayRaw::size = 0;
void FrontierManager::init(epic_ros::NodeHandle &nh, LIOInterface::Ptr &lio_interface,
                           TopoGraph::Ptr graph) {
  nh_ = nh;
  graph_ = graph;
  // External planning uses measured LiDAR air for both frontier views and guides.
  bool external;
  nh.param("fsm/external_local_planner",external,false);
  nh.param("global_planning/surface_continuity",surface_continuity_,external);
  nh.param("FrontierManager/preferred_ground_clearance",ground_clearance_,0.);
  lidar_map_interface_ = lio_interface;
  bool recovery;double hard,margin;
  nh.param("StartCorridorRecovery",recovery,external);
  nh.param("DilateRadiusHard",hard,.45);
  nh.param("StartCorridorRecoveryMargin",margin,.05);
  if (recovery && hard>0. && margin>=0.) departure_margin_=hard+margin;
  nh.param("FrontierManager/measured_aperture_clearance",aperture_clearance_,0.);
  if (aperture_clearance_<=hard+std::max(0.,margin)) aperture_clearance_=0.;
  // Inspection target policy is separate from the full 3-D collision map and
  // topology ROI. Defaults preserve the original EPIC mission.
  nh.param("FrontierManager/target_min_z", frtp_.target_min_z_, -1.0e6);
  nh.param("FrontierManager/target_max_z", frtp_.target_max_z_, 1.0e6);
  nh.param("FrontierManager/target_max_abs_normal_z", frtp_.target_max_abs_normal_z_, 1.0);
  nh.param("ViewpointManager/unreachable_retry_interval",vpp_.unreachable_retry_interval_,external ? 2. : 0.);
  nh.param("ViewpointManager/fixed_height_enabled", vpp_.fixed_height_enabled_, false);
  nh.param("ViewpointManager/fixed_height", vpp_.fixed_height_,external ? 1.7 : 1.75);
  nh.param("ViewpointManager/reached_position_tolerance", vpp_.reached_position_tolerance_,external ? .35 : .01);
  nh.param("ViewpointManager/reached_yaw_tolerance", vpp_.reached_yaw_tolerance_,external ? .2 : .01);
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
  lidar_map_interface_->lp_->persistent_hit_range_=surface_continuity_ ? frtp_.update_length_ : 0.;
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
  const auto entry = frtd_.label_map_.find(bytes);
  return entry == frtd_.label_map_.end() ? UNKNOWN : static_cast<CELL_STATE>(entry->second);
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
  omp_set_num_threads(epic_ros::planningWorkerCount(4));
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
  omp_set_num_threads(epic_ros::planningWorkerCount(4));
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
  unordered_set<ByteArrayRaw, ByteArrayRawHasher> queued;
  auto enqueue = [&](const PointType &pt, const Eigen::Vector3f &fallback) {
    if (get_state(pt) != FRONTIER_DIS && get_state(pt) != FRONTIER_DIR) return;
    ByteArrayRaw bytes; pos2bytes(pt, bytes);
    if (!queued.insert(bytes).second) return;
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
    if(pt.z<frtp_.target_min_z_ || pt.z>frtp_.target_max_z_ ||
        (frtp_.target_max_abs_normal_z_<1.f &&
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
    if (surface_continuity_ && observedCluster(*cluster)) continue;
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

// Keep a close-observation task for each measured surface patch. Completion
// requires a return from the same face while the body is alongside it.
bool FrontierManager::observedAir(const Eigen::Vector3f &point) const {
  if (!point.allFinite()) return false;
  const Eigen::Vector3i key=(point/frtp_.cell_size_).array().floor().cast<int>();
  return observed_air_.count(key)>0;
}

bool FrontierManager::observedGuide(const vector<Eigen::Vector3f> &path) const {
  if (path.size()<2) return false;
  const float step=.5f*frtp_.cell_size_;
  for (size_t i=1;i<path.size();++i) {
    const Eigen::Vector3f delta=path[i]-path[i-1];
    const int n=std::max(1,int(std::ceil(delta.norm()/step)));
    for (int j=0;j<=n;++j)
      if (!observedAir(path[i-1]+delta*(float(j)/n))) return false;
  }
  return true;
}

bool FrontierManager::withinFlightHeight(const Eigen::Vector3f &position) const {
  const auto &astar=graph_->parallel_bubble_astar_;
  return position.allFinite() && (!astar || (position.z()>=astar->flight_min_z_-1.e-5f &&
                                             position.z()<=astar->flight_max_z_+1.e-5f));
}

bool FrontierManager::certifyFlightGuide(const vector<Eigen::Vector3f> &path) const {
  if (certifyCruiseGuide(path)) return true;
  if (path.size()<2 || last_aperture_path_.size()<2) return false;
  // Clearance below cruise is specific to the checked crossing. Preserve it
  // through reanchoring and simplification only within its measured route.
  const float tolerance=.5f*frtp_.cell_size_;
  const auto on_crossing=[&](const Eigen::Vector3f &point) {
    for (size_t k=1;k<last_aperture_path_.size();++k) {
      const Eigen::Vector3f start=last_aperture_path_[k-1],edge=last_aperture_path_[k]-start;
      const float t=std::clamp((point-start).dot(edge)/std::max(.0001f,edge.squaredNorm()),0.f,1.f);
      if ((point-start-t*edge).squaredNorm()<=tolerance*tolerance) return true;
    }
    return false;
  };
  for (size_t i=1;i<path.size();++i) {
    const Eigen::Vector3f edge=path[i]-path[i-1];
    const int steps=std::max(1,int(std::ceil(edge.norm()/tolerance)));
    for (int j=0;j<=steps;++j) {
      if (!on_crossing(path[i-1]+edge*(float(j)/steps))) return false;
    }
  }
  return certifyShortAperture(path);
}

// A short constriction can connect two normal-clearance stances. Check every
// real segment at the aperture clearance and bound the distance spent below
// cruise clearance. The entering aircraft can also resume a checked crossing.
bool FrontierManager::certifyShortAperture(const vector<Eigen::Vector3f> &path) const {
  if (aperture_clearance_<=0. || !surface_continuity_ || path.size()<2 || !surface_body_) return false;
  const auto &astar=graph_->parallel_bubble_astar_;
  if (!astar || aperture_clearance_>=astar->safe_distance_) return false;
  const float cruise=astar->safe_distance_,radius=graph_->bubble_min_radius_;
  if ((path.front()-*surface_body_).norm()>.55f ||
      lidar_map_interface_->getDisToOcc(path.back())<cruise+.02f ||
      !std::all_of(path.begin(),path.end(),[&](const auto &p){return withinFlightHeight(p);}) ||
      !observedGuide(path)) return false;
  float tight_length=0.f,total=0.f;
  for (size_t i=1;i<path.size();++i) {
    const Eigen::Vector3f delta=path[i]-path[i-1];
    const float length=delta.norm();
    total+=length;
    if (total>frtp_.update_length_) return false;
    if (length<.001f) continue;
    vector<Eigen::Vector3f> edge{path[i-1],path[i]};
    if (!astar->collisionCheck_shortenPath(edge,aperture_clearance_)) return false;
    const int steps=std::max(1,int(std::ceil(length/(.25f*frtp_.cell_size_))));
    const float step=length/steps;
    for (int j=0;j<steps;++j) {
      const auto q=(path[i-1]+delta*((j+.5f)/steps)).eval();
      // Distance to an occupied point can decrease by at most half a step.
      if (lidar_map_interface_->getDisToOcc(q)<cruise+.5f*step) tight_length+=step;
      if (tight_length>3.f*radius) return false;
    }
  }
  return true;
}

bool FrontierManager::certifyCruiseGuide(const vector<Eigen::Vector3f> &path) const {
  if (!std::all_of(path.begin(),path.end(),[&](const auto &p){return withinFlightHeight(p);}) ||
      !observedGuide(path)) return false;
  auto astar=graph_->parallel_bubble_astar_;
  const float cruise=astar->safe_distance_;
  float achieved=lidar_map_interface_->getDisToOcc(path.front());
  bool recovering=achieved<=cruise+.01f;
  if (recovering && (departure_margin_<=0. || achieved<=departure_margin_+.01 ||
      !surface_body_ || (path.front()-*surface_body_).norm()>.55f)) return false;
  float escaped=0.f;
  for (size_t i=1;i<path.size();++i) {
    Eigen::Vector3f from=path[i-1];
    const Eigen::Vector3f delta=path[i]-from;
    const float length=delta.norm();
    if (length<.001f) continue;
    if (recovering) {
      const int parts=std::max(1,int(std::ceil(length/.15f)));
      for (int k=1;k<=parts && recovering;++k) {
        const Eigen::Vector3f next=path[i-1]+delta*(float(k)/parts);
        const float clear=lidar_map_interface_->getDisToOcc(next);
        vector<Eigen::Vector3f> segment{from,next};
        escaped+=length/parts;
        if (escaped>2.5f || clear<achieved-.06f ||
            !astar->collisionCheck_shortenPath(segment,departure_margin_)) return false;
        achieved=std::max(clear,achieved);from=next;
        recovering=clear<=cruise+.02f;
      }
    }
    if (!recovering && (path[i]-from).norm()>.001f) {
      vector<Eigen::Vector3f> edge{from,path[i]};
      if (!astar->collisionCheck_shortenPath(edge)) return false;
    }
  }
  return !recovering;
}

bool FrontierManager::connectObserved(const Eigen::Vector3f &start,const Eigen::Vector3f &goal,
                                      double budget,vector<Eigen::Vector3f> &route) const {
  auto astar=graph_->parallel_bubble_astar_;
  route={start,goal};
  if (lidar_map_interface_->IsPathInBox(route)) {
    if (observedGuide(route) && astar->collisionCheck_shortenPath(route)) return true;
    if (surface_body_ && (start-*surface_body_).norm()<.55f && certifyFlightGuide(route)) return true;
  }
  route.clear();
  if (budget<=0. || !observedAir(start) || !observedAir(goal)) return false;
  // Short direct joins dominate normal flight. For the remaining joins,
  // a narrow band of safe air can sit entirely between coarse grid centres.
  const auto begin=std::chrono::steady_clock::now();
  if (connectObservedGrid(start,goal,frtp_.cell_size_,.55*budget,route)) return true;
  const double remaining=budget-std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  return remaining>0. && connectObservedGrid(start,goal,.5f*frtp_.cell_size_,remaining,route);
}

bool FrontierManager::connectObservedGrid(const Eigen::Vector3f &start,const Eigen::Vector3f &goal,
                                         float cell,double budget,vector<Eigen::Vector3f> &route,
                                         bool distant) const {
  route.clear();
  auto astar=graph_->parallel_bubble_astar_;
  if (!start.allFinite() || !goal.allFinite() ||
      !lidar_map_interface_->IsInBox(start) || !lidar_map_interface_->IsInBox(goal)) return false;
  const float z_margin=distant ? std::max(cell,float(2.*astar->safe_distance_)) : cell;
  const float heuristic=distant ? 2.5f : 1.f;
  const auto index=[&](const Eigen::Vector3f &x)->Eigen::Vector3i {
    return (x/cell).array().floor().cast<int>();
  };
  const Eigen::Vector3i first=index(start),last=index(goal);
  if (first==last) return false;
  const auto point=[&](const Eigen::Vector3i &k)->Eigen::Vector3f {
    if (k==first) return start;
    if (k==last) return goal;
    return (k.cast<float>().array()+.5f)*cell;
  };
  struct Entry { Eigen::Vector3i key; float g,f; };
  const auto later=[](const Entry &a,const Entry &b){return a.f>b.f;};
  std::priority_queue<Entry,vector<Entry>,decltype(later)> open(later);
  std::unordered_map<Eigen::Vector3i,float,Vector3i_Hash> cost,space;
  std::unordered_map<Eigen::Vector3i,Eigen::Vector3i,Vector3i_Hash> parent;
  const auto radius=[&](const Eigen::Vector3i &k) {
    const auto old=space.find(k);
    if (old!=space.end()) return old->second;
    const float value=lidar_map_interface_->getDisToOcc(point(k))-astar->safe_distance_;
    space.emplace(k,value);return value;
  };
  if (radius(last)<=.01f) return false;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(budget);
  cost[first]=0.f;
  if (radius(first)>.01f) {
    open.push({first,0.f,(start-goal).norm()});
  } else {
    // The measured body can be inside the cruise margin after a tracking
    // deviation. Seed only edges that clear it under the normal departure test.
    if (!surface_body_ || (start-*surface_body_).norm()>.55f) return false;
    for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y) for (int z=-1;z<=1;++z) {
      if (!x && !y && !z) continue;
      const Eigen::Vector3i direction(x,y,z);
      for (int k=1;k*cell<=2.5f;++k) {
        const Eigen::Vector3i key=first+k*direction;
        const Eigen::Vector3f seed=point(key);
        const float distance=(seed-start).norm();
        if (distance>2.5f || !observedAir(seed)) break;
        const vector<Eigen::Vector3f> segment{start,seed};
        if (radius(key)<=.02f || !lidar_map_interface_->IsPathInBox(segment) || !certifyFlightGuide(segment))
          continue;
        cost[key]=distance;parent[key]=first;
        open.push({key,distance,distance+(goal-seed).norm()});
        break;
      }
    }
  }
  while (!open.empty() && std::chrono::steady_clock::now()<deadline) {
    const auto top=open.top();open.pop();
    if (top.g>cost.at(top.key)) continue;
    if (top.key==last) {
      for (auto k=last;k!=first;k=parent.at(k)) route.push_back(point(k));
      route.push_back(start);std::reverse(route.begin(),route.end());
      auto shortened=route;
      if (astar->collisionCheck_shortenPath(shortened) && lidar_map_interface_->IsPathInBox(shortened) &&
          observedGuide(shortened)) route.swap(shortened);
      return true;
    }
    const auto current=point(top.key);
    for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y) for (int z=-1;z<=1;++z) {
      if (!x && !y && !z) continue;
      const Eigen::Vector3i next=top.key+Eigen::Vector3i(x,y,z);
      const auto pos=point(next);
      if (!observedAir(pos)) continue;
      if (pos.z()<std::min(start.z(),goal.z())-z_margin || pos.z()>std::max(start.z(),goal.z())+z_margin ||
          pos.z()<astar->flight_min_z_ || pos.z()>astar->flight_max_z_ || !lidar_map_interface_->IsInBox(pos)) continue;
      const float edge=(pos-current).norm();
      const float new_cost=top.g+edge+.5f*std::abs(pos.z()-current.z());
      const auto known=cost.find(next);
      if (known!=cost.end() && known->second<=new_cost) continue;
      if (radius(next)<=.01f) continue;
      vector<Eigen::Vector3f> segment{current,pos};
      if (!lidar_map_interface_->IsPathInBox(segment) || !observedGuide(segment) ||
          (radius(next)+radius(top.key)<edge+.001f && !astar->collisionCheck_shortenPath(segment))) continue;
      parent[next]=top.key;cost[next]=new_cost;
      open.push({next,new_cost,new_cost+heuristic*(goal-pos).norm()});
    }
  }
  return false;
}

namespace {
// Classify the measured surface independently of the direction of the ray.
bool inspectionSurface(LIOInterface &map,const PointType &hit,float cell,float max_normal_z) {
  if (max_normal_z>=1.f) return true;
  PointVector neighbours;
  vector<float> squared;
  map.KNN(hit,12,neighbours,squared);
  const size_t count=std::upper_bound(squared.begin(),squared.end(),2.25f*cell*cell)-squared.begin();
  if (count<4) return false;
  Eigen::Vector3f mean=Eigen::Vector3f::Zero();
  for (size_t i=0;i<count;++i) mean+=neighbours[i].getVector3fMap();
  mean/=float(count);
  Eigen::Matrix3f covariance=Eigen::Matrix3f::Zero();
  for (size_t i=0;i<count;++i) {
    const Eigen::Vector3f offset=neighbours[i].getVector3fMap()-mean;
    covariance.noalias()+=offset*offset.transpose();
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> plane(covariance);
  if (plane.info()!=Eigen::Success) return false;
  const auto &variance=plane.eigenvalues();
  return variance[1]>.0025f*count*cell*cell && variance[0]<.25f*variance[1] &&
         std::abs(plane.eigenvectors()(2,0))<=max_normal_z;
}

unsigned char observedSide(const Eigen::Vector3f &toward) {
  const float threshold=.4f*toward.head<2>().norm();
  unsigned char mask=0;
  if (toward.x()>threshold) mask|=1;
  if (toward.x()<-threshold) mask|=2;
  if (toward.y()>threshold) mask|=4;
  if (toward.y()<-threshold) mask|=8;
  return mask;
}
}

void FrontierManager::recordFaceHit(const Eigen::Vector3f &sensor,const Eigen::Vector3f &hit) {
  const Eigen::Vector3f toward=sensor-hit;
  const float range=toward.norm(),close_range=frtp_.good_observation_trust_length_;
  if (range>std::max(close_range,std::min(2.f*close_range,frtp_.update_length_)) ||
      toward.head<2>().norm()<frtp_.cell_size_) return;
  const unsigned char side=observedSide(toward);
  const float resolution=2.f*frtp_.cell_size_;
  const Eigen::Vector3i patch=(hit/resolution).array().floor().cast<int>();
  auto &completed=inspected_faces_[patch];
  if ((completed&side)==side) return;
  const Eigen::Vector3i sub=((hit/resolution-patch.cast<float>())*4.f).array().floor().cast<int>().cwiseMax(0).cwiseMin(3);
  const uint64_t bit=uint64_t(1)<<(sub.x()+4*sub.y()+16*sub.z());
  auto &evidence=face_evidence_[patch];
  for (unsigned int i=0;i<4;++i) {
    const unsigned char face=1U<<i;
    if (!(side&face) || (completed&face)) continue;
    if (range<=close_range) {
      if (!evidence.occupied_subcells[i]) evidence.first_scan[i]=surface_scan_;
      evidence.occupied_subcells[i]|=bit;
      if (evidence.first_scan[i]!=surface_scan_ && __builtin_popcountll(evidence.occupied_subcells[i])>=4) {
        completed|=face;
        continue;
      }
    }
    // An oblique pass can measure a lower wall before the sensor is close
    // enough to see it again. Require denser returns across four scans at
    // longer range, always on the same face and in the same height band.
    evidence.all_subcells[i]|=bit;
    if (!evidence.scan_count[i] || evidence.last_scan[i]!=surface_scan_) {
      evidence.last_scan[i]=surface_scan_;
      if (evidence.scan_count[i]<4) ++evidence.scan_count[i];
    }
    if (evidence.scan_count[i]>=4 && __builtin_popcountll(evidence.all_subcells[i])>=10)
      completed|=face;
  }
}

bool FrontierManager::observedFace(const Eigen::Vector3f &point,const Eigen::Vector3f &normal) const {
  const unsigned char side=observedSide(normal);
  const float resolution=2.f*frtp_.cell_size_;
  const Eigen::Vector3f tangent(normal.y(),-normal.x(),0.f);
  // A raster contour or a frontier voxel is offset slightly from its actual
  // return. Probe the same face across that quantisation boundary.
  for (float depth:{0.f,-.5f,.5f}) for (float along:{0.f,-.5f,.5f}) {
    const Eigen::Vector3f q=point+frtp_.cell_size_*(depth*normal+along*tangent);
    const Eigen::Vector3i key=(q/resolution).array().floor().cast<int>();
    const auto found=inspected_faces_.find(key);
    if (found!=inspected_faces_.end() && (found->second&side)) return true;
  }
  return false;
}

bool FrontierManager::observedCluster(const ClusterInfo &cluster) const {
  if (cluster.cells_.empty()) {
    // A displaced first goal stores one signed wall sample per voxel instead
    // of native frontier cells. Apply the same completion tolerance but keep
    // any remainder large enough to form a frontier of its own.
    if (!cluster.displaced_issued_goal_ || cluster.contour_support_.empty()) return false;
    const size_t allowance=std::min(cluster.contour_support_.size()/5,
        size_t(std::max(1,frtp_.cluster_minmum_point_num_)-1));
    size_t unseen=0;
    for (const auto &s:cluster.contour_support_) {
      if (s.kind!=ObservationSample::Kind::WallFace) return false;
      if (!observedFace(s.point,s.toward_sensor) && ++unseen>allowance) return false;
    }
    return true;
  }
  // A few isolated raster-edge cells do not warrant another visit to a
  // well-measured face. A small new cluster still needs direct evidence.
  size_t unseen=0;
  const size_t allowance=cluster.cells_.size()/5;
  for (size_t i=0;i<cluster.cells_.size();++i) {
    const auto &side=i<cluster.norms_.size() ? cluster.norms_[i] : cluster.normal_;
    if (!observedFace(cluster.cells_[i].getVector3fMap(),side) && ++unseen>allowance) return false;
  }
  return true;
}

// Face evidence survives cluster regeneration and distinguishes the two sides
// of a shelf. Return unique cell keys so a two-leg preview can remove overlap.
std::vector<Eigen::Vector3i> FrontierManager::unseenSupport(const ClusterInfo &cluster) const {
  std::vector<Eigen::Vector3i> result;
  if (!cluster.contour_approach_.empty() && !cluster.contour_support_.empty()) {
    for (const auto &sample:cluster.contour_support_) {
      bool seen;
      switch (sample.kind) {
        case ObservationSample::Kind::UnknownAir:
          seen=observedAir(sample.point);
          break;
        case ObservationSample::Kind::AnyFace: {
          const Eigen::Vector3i patch=(sample.point/(2.f*frtp_.cell_size_)).array().floor().cast<int>();
          auto it=inspected_faces_.find(patch);
          seen=it!=inspected_faces_.end() && it->second;
          break;
        }
        default:
          seen=observedFace(sample.point,sample.toward_sensor);
      }
      if (!seen) result.push_back((sample.point/frtp_.cell_size_).array().floor().cast<int>());
    }
    return result;
  }
  for (const auto &cell : cluster.visible_cells_) {
    if (!observedFace(cell.getVector3fMap(), cluster.normal_))
      result.push_back((cell.getVector3fMap()/frtp_.cell_size_).array().floor().cast<int>());
  }
  return result;
}

// A saved sweep may have gathered evidence along a different approach.
// A trip back to its endpoint needs a remaining wall patch visible there.
bool FrontierManager::hasRevisitableSurface(const ClusterInfo &observation,const Eigen::Vector3f &stance) const {
  const float cell=frtp_.cell_size_;
  const float range=frtp_.good_observation_trust_length_;
  std::unordered_set<Eigen::Vector3i,Vector3i_Hash> patches,tested;
  const auto check=[&](const Eigen::Vector3f &expected,const Eigen::Vector3f *original_side) {
    if ((expected-stance).squaredNorm()>range*range) return false;
    PointVector hits;
    vector<float> distances;
    lidar_map_interface_->KNN(PointType(expected.x(),expected.y(),expected.z()),1,hits,distances);
    if (hits.empty() || distances.front()>cell*cell) return false;
    const Eigen::Vector3f hit=hits.front().getVector3fMap(),delta=stance-hit;
    const float length=delta.norm();
    if (length<=1.6f*cell || length>range) return false;
    const Eigen::Vector3f normal=delta/length;
    if ((original_side && original_side->dot(normal)<=0.f) || observedFace(hit,normal)) return false;
    const Eigen::Vector3i key=(hit/cell).array().floor().cast<int>();
    if (!tested.insert(key).second || !inspectionSurface(*lidar_map_interface_,hits.front(),cell,frtp_.target_max_abs_normal_z_))
      return false;
    vector<Eigen::Vector3f> sight{stance,hit+1.6f*cell*normal};
    if (!observedGuide(sight) || !graph_->parallel_bubble_astar_->collisionCheck_shortenPath(sight,.2f*cell)) return false;
    for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y) for (int z=-1;z<=1;++z)
      if (patches.count(key+Eigen::Vector3i(x,y,z))) return true;
    patches.insert(key);
    return false;
  };
  for (const auto &s:observation.contour_support_)
    if (s.kind==ObservationSample::Kind::WallFace && check(s.point,&s.toward_sensor)) return true;
  for (const auto &p:observation.visible_cells_)
    if (check(p.getVector3fMap(),nullptr)) return true;
  return false;
}

// Build a signed observation at a previously issued stance from actual nearby returns.
bool FrontierManager::measureUnseenWall(const Eigen::Vector3f &stance,ClusterInfo &observation) const {
  const float radius=graph_->bubble_min_radius_,cell=frtp_.cell_size_;
  const Eigen::Vector3f extent(3.f*radius,3.f*radius,1.5f*radius);
  PointVector returns;
  lidar_map_interface_->boxSearch(stance-extent,stance+extent,returns);
  std::unordered_set<Eigen::Vector3i,Vector3i_Hash> voxels;
  for (const auto &hit:returns) {
    const Eigen::Vector3f p=hit.getVector3fMap(),delta=stance-p;
    const float length=delta.norm();
    if (length<1.6f*cell || std::abs(delta.z())>.7f*length) continue;
    const Eigen::Vector3f side=delta/length;
    const Eigen::Vector3i key=(p/cell).array().floor().cast<int>();
    if (!voxels.insert(key).second || observedFace(p,side) ||
        !inspectionSurface(*lidar_map_interface_,hit,cell,frtp_.target_max_abs_normal_z_)) continue;
    observation.contour_support_.push_back({ObservationSample::Kind::WallFace,p,side});
  }
  return hasRevisitableSurface(observation,stance);
}

// A horizontal exploration goal needs a visible upright surface or an opening
// beside one. Unflown volume above an already surveyed floor is not a task.
bool FrontierManager::hasLateralSurveyGain(const Eigen::Vector3f &stance,bool following_wall) const {
  if (frtp_.target_max_abs_normal_z_>=1.f) return true;
  const float cell=frtp_.cell_size_,radius=graph_->bubble_min_radius_;
  const float range=following_wall ? rightWallSearchRange() : 3.f*radius;
  PointVector hits;
  const Eigen::Vector3f extent(range,range,cell);
  lidar_map_interface_->boxSearch(stance-extent,stance+extent,hits);
  bool beside_wall=false;
  std::unordered_set<Eigen::Vector3i,Vector3i_Hash> tested;
  for (const auto &hit:hits) {
    const Eigen::Vector3f point=hit.getVector3fMap(),offset=stance-point;
    const float length=offset.norm();
    if (length<=cell || length>range) continue;
    const Eigen::Vector3i key=(point/cell).array().floor().cast<int>();
    if (!tested.insert(key).second || !inspectionSurface(*lidar_map_interface_,hit,cell,frtp_.target_max_abs_normal_z_)) continue;
    const Eigen::Vector3f normal=offset/length;
    vector<Eigen::Vector3f> sight{stance,point+std::min(1.6f*cell,length)*normal};
    if (!observedGuide(sight) || !graph_->parallel_bubble_astar_->collisionCheck_shortenPath(sight,.2f*cell)) continue;
    beside_wall=true;
    if (following_wall || !observedFace(point,normal)) return true;
  }
  if (!beside_wall) return false;
  // A seen door jamb can still lead to a room. Require a lateral gap
  // at least one vehicle diameter deep across adjacent lidar bearings.
  constexpr int bearings=16;
  bool first=false,previous=false;
  for (int i=0;i<bearings;++i) {
    const float angle=i*float(2.*M_PI/bearings);
    const Eigen::Vector3f ray(std::cos(angle),std::sin(angle),0.f);
    float depth=0.f;
    for (float d=cell;d<=frtp_.good_observation_trust_length_;d+=cell) {
      const Eigen::Vector3f q=stance+d*ray;
      if (!lidar_map_interface_->IsInBox(q) || lidar_map_interface_->getDisToOcc(q)<measuredStanceClearance()) break;
      depth=observedAir(q) ? 0.f : depth+cell;
      if (depth>=2.f*radius) break;
    }
    const bool open=depth>=2.f*radius;
    if (previous && open) return true;
    if (i==0) first=open;
    previous=open;
  }
  return first && previous;
}

// Sampling a stance does not visit it. A handoff or a later scan may reject
// it. Only actual flight and surface measurements suppress it indefinitely.
void FrontierManager::expireOpeningAttempts(double now) {
  for (auto it=attempted_sightlines_.begin();it!=attempted_sightlines_.end();) {
    if (it->second<=now) it=attempted_sightlines_.erase(it);
    else ++it;
  }
}
void FrontierManager::recordOpeningAttempt(const Eigen::Vector3i &key,double now) {
  attempted_sightlines_[key]=now+std::max(double(vpp_.unreachable_retry_interval_),.5);
}

// A viewpoint in measured air can stand in the same envelope used to
// certify its incoming flight. Sparse topology nodes retain their own radius.
float FrontierManager::measuredStanceClearance() const {
  return graph_->parallel_bubble_astar_ ? graph_->parallel_bubble_astar_->safe_distance_+.025f :
      graph_->bubble_min_radius_;
}

std::optional<Eigen::Vector2f> FrontierManager::recentFlightDirection(const Eigen::Vector3f &body) const {
  for (auto it=flight_history_.rbegin();it!=flight_history_.rend();++it) {
    const Eigen::Vector2f travel=(body-*it).head<2>();
    if (travel.norm()>=2.f*graph_->bubble_min_radius_) return travel.normalized();
  }
  return {};
}

// Fit the incoming and outgoing legs of a recently flown level turn. The
// crossing lies on the flight track. An opening is accepted by the tour only
// when a proposed exit actually recrosses here while a fresh wall remains.
std::optional<std::pair<Eigen::Vector3f,Eigen::Vector2f>> FrontierManager::recentRightTurn(const Eigen::Vector3f &body) const {
  const float radius=graph_->bubble_min_radius_;
  std::optional<std::pair<Eigen::Vector3f,Eigen::Vector2f>> result;
  float best=.60f;
  for (auto pivot=flight_history_.rbegin();pivot!=flight_history_.rend();++pivot) {
    const Eigen::Vector3f outward=body-*pivot;
    const float length=outward.head<2>().norm();
    if (length>6.f*radius || std::abs(outward.z())>radius) break;
    if (length<2.f*radius) continue;
    for (auto prior=std::next(pivot);prior!=flight_history_.rend();++prior) {
      const Eigen::Vector3f approach=*pivot-*prior;
      if (std::abs(approach.z())>radius) break;
      if (approach.head<2>().norm()<5.f*radius) continue;
      const Eigen::Vector2f right=Eigen::Vector2f(approach.y(),-approach.x()).normalized();
      const float turn=outward.head<2>().dot(right)/length;
      if (turn>best) {
        best=turn;
        result=std::make_pair((*pivot+.4f*outward).eval(),right);
      }
      break;
    }
  }
  return result;
}

// A raised stance in an open column above the same measured floor is still
// part of the present survey band. A rising surface or an intervening slab
// leaves the next elevation available for exploration.
bool FrontierManager::aboveSameSurveyFloor(const Eigen::Vector3f &goal) const {
  if (!survey_height_ || !surface_body_ || ground_clearance_<=0.) return false;
  const float radius=graph_->bubble_min_radius_,cell=frtp_.cell_size_;
  if (goal.z()<=*survey_height_+radius || goal.z()<=surface_body_->z()+.25f*cell) return false;
  const auto level=measuredSurveyHeight(goal,float(ground_clearance_));
  if (!level || std::abs(*level-*survey_height_)>cell) return false;
  Eigen::Vector3f lower=goal;
  lower.z()=*level;
  vector<Eigen::Vector3f> column{lower,goal};
  return observedGuide(column) && graph_->parallel_bubble_astar_->collisionCheck_shortenPath(column,radius*.5f);
}

// Ascending during recovery needs a measured rise in the supporting floor.
// A high unobserved patch on a previously traversed wall cannot establish it.
bool FrontierManager::followsRisingFloor(const vector<Eigen::Vector3f> &route) const {
  if (route.size()<2) return false;
  const float radius=graph_->bubble_min_radius_,cell=frtp_.cell_size_;
  const auto level=currentSurveyBand(route.front());
  if (!level || route.back().z()<=*level+radius || ground_clearance_<=0.) return true;
  const float rise=route.back().z()-*level;
  const auto on_floor=[&](const Eigen::Vector3f &p) {
    if (p.z()<*level+.5f*rise) return false;
    const auto support=measuredSurveyHeight(p,ground_clearance_);
    return support && *support>*level+.5f*rise && std::abs(*support-p.z())<=radius;
  };
  if (on_floor(route.back())) return true;
  for (size_t i=route.size()-1;i>0;--i) {
    const Eigen::Vector3f edge=route[i-1]-route[i];
    const int count=std::max(1,int(std::ceil(edge.norm()/std::max(cell,radius))));
    for (int j=1;j<=count;++j)
      if (on_floor(route[i]+edge*(float(j)/count))) return true;
  }
  return false;
}

// An unflown voxel above a measured pass can be just the open space above
// a wall. When both heights see the same pass, require observable surface
// gain at the upper stance. A floor separating two levels breaks this test.
bool FrontierManager::supportsElevatedOpening(const Eigen::Vector3f &goal) const {
  const float radius=graph_->bubble_min_radius_,cell=frtp_.cell_size_;
  const float trust=frtp_.good_observation_trust_length_;
  if (!survey_height_ || goal.z()<=*survey_height_+radius) return true;
  if (aboveSameSurveyFloor(goal)) return false;
  Eigen::Vector3f lower=goal;
  lower.z()=*survey_height_;
  if (!visitedStance(lower) && !nearFlightPath(lower,trust,0.f,.2f*cell) &&
      (!surface_body_ || !surveyedFromAround(*surface_body_,lower))) return true;
  vector<Eigen::Vector3f> column{lower,goal};
  if (!graph_->parallel_bubble_astar_->collisionCheck_shortenPath(column,.2f*cell)) return true;

  PointVector hits;
  lidar_map_interface_->boxSearch(goal-Eigen::Vector3f(trust,trust,radius),
                                  goal+Eigen::Vector3f::Constant(trust),hits);
  std::unordered_set<Eigen::Vector3i,Vector3i_Hash> visible;
  for (const auto &hit:hits) {
    const Eigen::Vector3f point=hit.getVector3fMap(),delta=point-goal;
    const float range=delta.norm();
    if (range<=1.6f*cell || range>trust) continue;
    const Eigen::Vector3i patch=(point/(2.f*cell)).array().floor().cast<int>();
    if (visible.count(patch) || observedFace(point,-delta/range) ||
        !inspectionSurface(*lidar_map_interface_,hit,cell,frtp_.target_max_abs_normal_z_)) continue;
    vector<Eigen::Vector3f> ray{goal,point-1.6f*cell*delta/range};
    if (observedGuide(ray) && graph_->parallel_bubble_astar_->collisionCheck_shortenPath(ray,.2f*cell)) {
      visible.insert(patch);
      if (visible.size()>=3) return true;
    }
  }
  return false;
}

// Search the connected component of measured air when the local observations
// cease to move the body. Every selected edge is checked again at handoff.
bool FrontierManager::reachableOpening(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route,
                                       const Eigen::Vector3f *toward) {
  route.clear();
  surface_body_=body;
  if (!observedAir(body)) return false;
  const double now=epic_ros::Time::now().toSec();
  if (!toward) expireOpeningAttempts(now);
  using Key=Eigen::Vector3i;
  const float safe=graph_->parallel_bubble_astar_->safe_distance_;
  const auto keyOf=[&](const Eigen::Vector3f &p)->Key {
    return (p/(2.f*frtp_.cell_size_)).array().floor().cast<int>();
  };
  float best=-std::numeric_limits<float>::infinity();
  float novel=0.f;
  const auto search=[&](float cell) {
    const Key start=(body/cell).array().floor().cast<int>();
    const auto position=[&](const Key &k)->Eigen::Vector3f {
      if (k==start) return body;
      return (k.cast<float>().array()+.5f)*cell;
    };
    struct Entry {Key key;float cost;};
    const auto later=[](const Entry &a,const Entry &b){return a.cost>b.cost;};
    std::priority_queue<Entry,vector<Entry>,decltype(later)> open(later);
    std::unordered_map<Key,float,Vector3i_Hash> cost,clearance,unflown;
    std::unordered_map<Key,Key,Vector3i_Hash> parent;
    const auto clear=[&](const Key &k) {
      auto it=clearance.find(k);
      if (it!=clearance.end()) return it->second;
      return clearance.emplace(k,lidar_map_interface_->getDisToOcc(position(k))).first->second;
    };
    std::vector<std::pair<float,Key>> candidates;
    cost[start]=0.f;unflown[start]=0.f;open.push({start,0.f});
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(60);
    size_t expanded=0;
    while (!open.empty() && expanded<1800 && std::chrono::steady_clock::now()<deadline) {
      const auto current=open.top();open.pop();
      if (current.cost>cost.at(current.key)) continue;
      ++expanded;
      const auto from=position(current.key);
      const float distance=(from-body).norm();
      int exposed=0;
      if (!toward) for (int axis=0;axis<3;++axis) for (int sign:{-1,1}) {
        Key direction=Key::Zero();direction[axis]=sign;
        const auto offset=direction.cast<float>()*frtp_.cell_size_;
        if (!observedAir(from+offset) && !observedAir(from+2.f*offset) &&
            clear(current.key+direction)>safe) ++exposed;
      }
      // A lidar can see the air around a bend before the vehicle can see
      // beyond it. Travel through a continuous unflown part of that component
      // can reveal a new face even when its final voxel has no unknown neighbour.
      const float fresh_length=unflown.at(current.key);
      if (toward) {
        // Previously flown air is productive during a return. Advance toward
        // the destination through the current measured component when an old
        // recorded stance ends immediately after the aircraft.
        const float progress=(*toward-body).norm()-(*toward-from).norm();
        if (progress>2.f*safe) candidates.emplace_back(progress-.1f*current.cost,current.key);
      } else if ((exposed || fresh_length>2.5f*safe) && distance>safe && !attempted_sightlines_.count(keyOf(from))) {
        const bool fresh=!visitedStance(from);
        // Two elevations in a sloping passage may share a single horizontal
        // visitation band. Require fine-scale separation from the actual flight.
        if (fresh || (std::abs(from.z()-body.z())>safe && !nearFlightPath(from,.65f*safe,0.f,.2f))) {
          const float height_cost=survey_height_ ? .45f*std::abs(from.z()-*survey_height_) : 0.f;
          const float reward=std::min(distance,5.f*safe)+.6f*exposed+std::min(clear(current.key),2.f*safe)
              +.4f*std::min(fresh_length,4.f*safe)-height_cost;
          candidates.emplace_back(reward-.32f*current.cost,current.key);
        }
      }
      for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y) for (int z=-1;z<=1;++z) {
        if (x==0 && y==0 && z==0) continue;
        const Key next=current.key+Key(x,y,z);
        const auto to=position(next);
        if (!observedAir(to)) continue;
        const float length=(to-from).norm(),next_cost=current.cost+length+.35f*std::abs(to.z()-from.z());
        if (next_cost>frtp_.update_length_ || !isInBox(to) || to.z()<graph_->parallel_bubble_astar_->flight_min_z_ ||
            to.z()>graph_->parallel_bubble_astar_->flight_max_z_) continue;
        const auto old=cost.find(next);
        if (old!=cost.end() && old->second<=next_cost) continue;
        if (clear(next)<=safe+.02f) continue;
        if (!observedGuide({from,to})) continue;
        if (clear(next)+clear(current.key)-2.f*safe<length && !certifyFlightGuide({from,to})) continue;
        cost[next]=next_cost;
        unflown[next]=visitedStance(to) ? 0.f : fresh_length+length;
        parent[next]=current.key;open.push({next,next_cost});
      }
    }
    std::sort(candidates.begin(),candidates.end(),[](const auto &a,const auto &b){return a.first>b.first;});
    int unsupported=0;
    for (const auto &candidate:candidates) {
      if (candidate.first<=best) break;
      if (!toward && (!supportsElevatedOpening(position(candidate.second)) ||
          !hasLateralSurveyGain(position(candidate.second)))) {
        if (++unsupported>=12) break;
        continue;
      }
      vector<Eigen::Vector3f> path;
      for (Key k=candidate.second;k!=start;k=parent.at(k)) path.push_back(position(k));
      path.push_back(body);std::reverse(path.begin(),path.end());
      if (!certifyFlightGuide(path)) continue;
      vector<Eigen::Vector3f> shortened{body};
      for (size_t i=0;i+1<path.size();) {
        size_t j=i+1;
        while (j+1<path.size() && certifyFlightGuide({path[i],path[j+1]})) ++j;
        shortened.push_back(path[j]);i=j;
      }
      route=std::move(shortened);
      best=candidate.first;
      novel=unflown.at(candidate.second);
      return true;
    }
    return false;
  };
  // A narrow continuous band of measured free air can pass between adjacent
  // coarse voxel centres. Refine when the coarse route fails to lead
  // across a sustained stretch of new space. Both passes use the same observed-air and collision checks.
  search(frtp_.cell_size_);
  if (route.empty() || (!toward && novel<2.5f*safe) || (route.back()-body).norm()<3.f*safe)
    search(.5f*frtp_.cell_size_);
  if (route.empty()) return false;
  if (!toward) {
    recordOpeningAttempt(keyOf(route.back()),now);
    ROS_INFO("Selected reachable opening %.2fm from body, %.2fm unflown", (route.back()-body).norm(),novel);
  }
  return true;
}

// Near a wall the measured vehicle centre can lie between the tracking
// margin and the cruise margin. Search outward from that actual position,
// then join the next observation to the same continuous flight guide.
bool FrontierManager::clearanceDepartureGuide(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route,
                                             const Eigen::Vector3f *return_goal) {
  route.clear();
  const float safe=graph_->parallel_bubble_astar_->safe_distance_,cell=frtp_.cell_size_;
  const float current=lidar_map_interface_->getDisToOcc(body);
  if (departure_margin_<=0. || current<=departure_margin_+.01f || current>safe+.02f ||
      !observedAir(body)) return false;
  surface_body_=body;
  Eigen::Vector3f seed=body;
  float best=-std::numeric_limits<float>::infinity();
  const auto consider=[&](const Eigen::Vector3f &q) {
    const auto &astar=graph_->parallel_bubble_astar_;
    if (!isInBox(q) || !observedAir(q) || q.z()<astar->flight_min_z_ || q.z()>astar->flight_max_z_) return;
    const float clearance=lidar_map_interface_->getDisToOcc(q);
    if (clearance<safe+.5f*cell) return;
    const float score=std::min(clearance,safe+cell)-.09f*(q-body).norm()-.06f*std::abs(q.z()-body.z());
    if (score>best && certifyFlightGuide({body,q})) {best=score;seed=q;}
  };
  for (float r:{safe,1.5f*safe,2.f*safe}) for (int i=0;i<16;++i) {
    const float yaw=float(2.*M_PI*i/16.);
    for (float dz:{0.f,-cell,cell})
      consider(body+Eigen::Vector3f(r*std::cos(yaw),r*std::sin(yaw),dz));
  }
  for (float dz:{-safe,safe}) consider(body+Eigen::Vector3f(0.f,0.f,dz));
  if (!std::isfinite(best)) return false;
  vector<Eigen::Vector3f> onward;
  const auto join=[&](bool linked,bool retracing) {
    surface_body_=body;
    route={body,seed};
    if (!linked || onward.size()<2 || (onward.back()-body).norm()<2.f*safe) return false;
    if (retracing) {
      if (!return_goal || (body-*return_goal).norm()-(onward.back()-*return_goal).norm()<1.5f*safe) return false;
    } else if (nearFlightPath(onward.back(),2.f*safe,0.f)) return false;
    route.insert(route.end(),onward.begin()+1,onward.end());
    if (certifyFlightGuide(route)) return true;
    route.resize(2);
    return false;
  };
  // Known flight history is valuable on return. A visited waypoint may
  // extend this departure only if it also makes measurable progress home.
  bool extended=join(reachableOpening(seed,onward),false);
  if (!extended) extended=join(observedBoundaryGuide(seed,onward),false);
  if (!extended && return_goal && !join(observedBoundaryGuide(seed,onward,nullptr,return_goal),true))
    join(reachableOpening(seed,onward,return_goal),true);
  ROS_INFO("Connected measured departure from %.2fm clearance to %.2fm",current,
           lidar_map_interface_->getDisToOcc(seed));
  return true;
}

// A coarse search from the body can miss a passage whose clear cross-section
// lies between grid centres. Seed the existing measured-space connector from
// the other end before declaring a return complete.
int FrontierManager::unknownBoundaryFaces(const Eigen::Vector3i &key,vector<ObservationSample> *samples) {
  const float cell=frtp_.cell_size_;
  const Eigen::Vector3f q=(key.cast<float>().array()+.5f)*cell;
  int faces=0;
  for (int axis=0;axis<3;++axis) for (int sign:{-1,1}) {
    Eigen::Vector3i step=Eigen::Vector3i::Zero();step[axis]=sign;
    if (observed_air_.count(key+step) || observed_air_.count(key+2*step)) continue;
    const Eigen::Vector3f direction=step.cast<float>()*cell;
    const Eigen::Vector3f beyond=q+2.f*direction;
    if (isInBox(beyond) && lidar_map_interface_->getDisToOcc((q+direction).eval())>cell &&
        lidar_map_interface_->getDisToOcc(beyond)>cell) {
      ++faces;
      if (samples) samples->push_back({ObservationSample::Kind::UnknownAir,q+direction,-direction});
    }
  }
  return faces;
}

bool FrontierManager::observedBoundaryGuide(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route,
                                           const Eigen::Vector2f *forward,const Eigen::Vector3f *toward,float minimum_forward) {
  return observedBoundaryGuide(body,route,forward,toward,minimum_forward,nullptr);
}

bool FrontierManager::observedBoundaryGuide(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route,
                                           const Eigen::Vector2f *forward,const Eigen::Vector3f *toward,float minimum_forward,
                                           vector<Eigen::Vector3f> *prefix,std::optional<float> survey_level,
                                           std::optional<float> fork_distance,bool ascending) {
  route.clear();
  if (prefix) prefix->clear();
  surface_body_=body;
  const float cell=frtp_.cell_size_,radius=graph_->bubble_min_radius_;
  const float trust=frtp_.good_observation_trust_length_;
  const auto astar=graph_->parallel_bubble_astar_;
  struct Candidate {Eigen::Vector3f position;float score;};
  vector<Candidate> candidates;
  // A verified vertical free column over the current floor cannot contain
  // a second landing below its verified top. Keep certificates local to
  // this search so new scans or a change of floor invalidate them.
  std::unordered_map<Eigen::Vector3i,float,Vector3i_Hash> same_floor_top;
  for (const auto &key:observed_air_) {
    const Eigen::Vector3f q=(key.cast<float>().array()+.5f)*cell;
    if (survey_level && std::abs(q.z()-*survey_level)>radius) continue;
    if (ascending && (q.z()<=body.z()+radius || q.z()>body.z()+trust)) continue;
    if (q.z()<astar->flight_min_z_ || q.z()>astar->flight_max_z_) continue;
    const float range=(q-body).norm();
    if (range<2.f*radius || range>((survey_level || ascending) ? 2.f : 1.f)*frtp_.update_length_) continue;
    if ((!toward && visited_air_.count(key)) || !isInBox(q)) continue;
    const Eigen::Vector2f travel=(q-body).head<2>();
    // Search the advancing half of the measured component when evaluating
    // a reversal. A better-scoring point behind cannot hide a viable exit ahead.
    if (forward && travel.dot(*forward)<=.35f*travel.norm()*forward->norm()) continue;
    if (fork_distance && forward) {
      const Eigen::Vector2f right(forward->y(),-forward->x());
      if (travel.dot(right)<1.5f*radius || std::abs(travel.dot(*forward)-*fork_distance)>.75f*trust) continue;
    }
    if (minimum_forward>0.f && forward) {
      const Eigen::Vector2f axis=forward->normalized();
      const float advance=travel.dot(axis);
      if (advance<minimum_forward || std::abs(q.z()-body.z())>radius ||
          (travel-advance*axis).norm()>radius+.35f*advance) continue;
    }
    if (toward) {
      const float progress=(*toward-body).norm()-(*toward-q).norm();
      if (progress>2.f*radius && lidar_map_interface_->getDisToOcc(q)>radius+.02f)
        candidates.push_back({q,progress-.17f*range-.35f*std::abs(q.z()-body.z())});
      continue;
    }
    const int faces=unknownBoundaryFaces(key);
    if (!faces) continue;
    if (ascending && survey_height_ && q.z()>*survey_height_+radius) {
      const Eigen::Vector3i column(key.x(),key.y(),0);
      const auto top=same_floor_top.find(column);
      if (top!=same_floor_top.end() && q.z()<=top->second) continue;
      if (aboveSameSurveyFloor(q)) {
        same_floor_top[column]=q.z();
        continue;
      }
    }
    const float clearance=lidar_map_interface_->getDisToOcc(q);
    if (clearance<(ascending || survey_level || (forward && minimum_forward>0.f) ? measuredStanceClearance() : radius+.02f)) continue;
    float separation=trust*trust;
    for (const auto &past:flight_history_) {
      separation=std::min(separation,(q-past).squaredNorm());
      if (separation<4.f*radius*radius) break;
    }
    if (separation<4.f*radius*radius) continue;
    const float score=std::sqrt(separation)+.65f*(std::min(3,faces)*radius+std::min(2.f*radius,clearance))-.17f*range;
    candidates.push_back({q,score});
  }
  std::sort(candidates.begin(),candidates.end(),[](const Candidate &a,const Candidate &b) {
    if (a.score!=b.score) return a.score>b.score;
    if (a.position.x()!=b.position.x()) return a.position.x()<b.position.x();
    if (a.position.y()!=b.position.y()) return a.position.y()<b.position.y();
    return a.position.z()<b.position.z();
  });
  vector<Eigen::Vector3f> tested;
  float best_progress=0.f;
  int connections=0,unsupported=0;
  // Continue a bounded search after a failed solve at the same departure.
  // The score order is deterministic and the body can be stationary.
  if (ascending && (!ascent_search_ || (body-ascent_search_->first).norm()>radius))
    ascent_search_=std::make_pair(body,size_t(0));
  const size_t first=ascending && !candidates.empty() ? ascent_search_->second%candidates.size() : 0;
  for (size_t i=0;i<candidates.size();++i) {
    const size_t index=(first+i)%candidates.size();
    const auto &candidate=candidates[index];
    if (ascending) ++ascent_search_->second;
    const auto &q=candidate.position;
    // Nearby ascending cells can have different stair connections. A failed
    // connection certifies only its own endpoint.
    if (!ascending) {
      if (std::any_of(tested.begin(),tested.end(),[&](const auto &last){return (last-q).norm()<2.f*radius;})) continue;
      tested.push_back(q);
    }
    if (!toward && (!supportsElevatedOpening(q) || (!ascending && !hasLateralSurveyGain(q)))) {
      if (++unsupported>=12) break;
      continue;
    }
    vector<Eigen::Vector3f> partial,flown;
    if ((connectObserved(body,q,.025,route) ||
         ((survey_level || ascending) && connectFlightHistory(body,q,.025,route,ascending ? &flown : nullptr)) ||
         connectDistantObserved(body,q,route,prefix ? &partial : nullptr)) &&
        (!survey_level || std::all_of(route.begin(),route.end(),[&](const auto &p){
          return withinLevelApproach(p.z(),body.z(),*survey_level,radius);
        })) &&
        (!ascending || !revisitsLowerPass(body,route)) && certifyFlightGuide(route)) {
      if (!ascending && !toward && !forward && surface_continuity_) preferRightBranch(body,route,false);
      if (ascending) {ascent_search_.reset();untested_ascent_=false;}
      ROS_INFO("Connected unflown observation on measured-air boundary %.2fm away",(route.back()-body).norm());
      return true;
    }
    if (prefix) for (auto *leg:{&partial,&flown}) {
      if (leg->size()<2) continue;
      const float progress=(body-q).norm()-(leg->back()-q).norm();
      if (progress<=best_progress || (ascending && (progress<2.f*radius ||
          revisitsLowerPass(body,*leg) || !certifyFlightGuide(*leg)))) continue;
      best_progress=progress;
      *prefix=std::move(*leg);
    }
    if (++connections>=6) break;
  }
  if (ascending) untested_ascent_=ascent_search_->second<candidates.size();
  if (ascending && std::getenv("EPIC_TRACE_WALL_CHOICE"))
    ROS_INFO("Ascent search at %.2f %.2f %.2f: %zu candidates, %d unsupported, %d connections tried, %.2fm prefix",
             body.x(),body.y(),body.z(),candidates.size(),unsupported,connections,best_progress);
  route.clear();
  return false;
}

bool FrontierManager::withinLevelApproach(float height,float body_height,float survey_height,float radius) {
  // The floor-based preferred altitude can shift by one voxel while the body
  // follows a lower measured opening. Count the connection from either height.
  return height>=std::min(body_height,survey_height)-radius && height<=std::max(body_height,survey_height)+radius;
}

std::optional<float> FrontierManager::currentSurveyBand(const Eigen::Vector3f &body) const {
  // A floor can briefly disappear beneath the lidar at a doorway. The
  // measured vehicle height still anchors a search for a level flight path.
  if (!survey_height_) return ground_clearance_>0. ? std::optional<float>(body.z()) : std::nullopt;
  const float cell=frtp_.cell_size_;
  return body.z()+std::clamp(*survey_height_-body.z(),-cell,cell);
}

bool FrontierManager::pendingLevelGuide(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route) {
  const auto band=currentSurveyBand(body);
  if (!band) {route.clear();return false;}
  return observedBoundaryGuide(body,route,nullptr,nullptr,0.f,nullptr,band);
}

// A safe connector can retrace the stairs to a previously surveyed level even
// when its final viewpoint is fresh. Check the flown volume along its edges.
std::optional<float> FrontierManager::retiredSurveyCeiling() const {
  if (!lowest_flown_band_ || !highest_flown_band_ || ground_clearance_<=0.) return {};
  const float separation=std::max(2.*graph_->bubble_min_radius_,ground_clearance_);
  if (*highest_flown_band_-*lowest_flown_band_<2.f*separation) return {};
  return *highest_flown_band_-separation;
}

bool FrontierManager::revisitsLowerPass(const Eigen::Vector3f &body,const vector<Eigen::Vector3f> &route) const {
  if (!surface_continuity_ || ground_clearance_<=0. || route.empty()) return false;
  const float radius=graph_->bubble_min_radius_,cell=frtp_.cell_size_;
  if (const auto retired=retiredSurveyCeiling()) {
    // Permit level flight and recovery upward when the vehicle is already low.
    const float minimum=std::min(*retired,body.z()-.5f*cell);
    if (std::any_of(route.begin(),route.end(),[&](const auto &q){return q.z()<minimum;})) return true;
  }
  float lower=body.z()-std::max(2.f*radius,float(ground_clearance_)+cell);
  // Successive shorter guides can lead back down the same staircase.
  // Retain the established flight band across those replans. Level flight
  // and an ascent remain possible even after a local height correction.
  if (highest_flown_band_) {
    const float margin=std::max(1.5f*radius,float(ground_clearance_)-cell);
    lower=std::min(body.z()-cell,std::max(lower,*highest_flown_band_-margin));
  }
  Eigen::Vector3f start=route.front();
  for (const auto &end:route) {
    if (std::min(start.z(),end.z())<=lower) {
      const Eigen::Vector3f delta=end-start;
      const int steps=std::max(1,int(std::ceil(delta.norm()/cell)));
      for (int i=0;i<=steps;++i) {
        const Eigen::Vector3f q=start+(float(i)/steps)*delta;
        if (q.z()<=lower && (visitedStance(q) || nearFlightPath(q,2.f*radius,0.f,.25f*cell))) return true;
      }
    }
    start=end;
  }
  return false;
}

// A cached transfer can reach a fork many scans after it was selected.
// At an approaching left turn, compare the newly visible right and straight
// departures in the incoming flight frame before handing the transfer on.
bool FrontierManager::continueBeforeLeftTurn(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route) {
  const auto heading=recentFlightDirection(body);
  if (!heading || !surface_continuity_ || route.size()<2) return false;
  const float radius=graph_->bubble_min_radius_;
  const Eigen::Vector2f right(heading->y(),-heading->x());
  float arc=0.f,advance=0.f;
  for (size_t i=1;i<route.size();++i) {
    const Eigen::Vector2f offset=(route[i]-body).head<2>();
    arc+=(route[i]-route[i-1]).norm();
    if (arc>frtp_.update_length_ || std::abs(route[i].z()-body.z())>radius ||
        offset.dot(*heading)<-radius || offset.dot(right)>1.5f*radius) return false;
    advance=std::max(advance,offset.dot(*heading));
    if (offset.dot(right)>=-1.5f*radius) continue;
    vector<Eigen::Vector3f> bend(route.begin(),route.begin()+i+1);
    if (preferRightBranch(body,bend,false)) {route.swap(bend);return true;}
    vector<Eigen::Vector3f> straight;
    if (!observedBoundaryGuide(body,straight,&*heading,nullptr,advance+2.f*radius,nullptr,currentSurveyBand(body)))
      return false;
    // A distant left arm can also fall inside the forward cone. Require
    // the actual measured route to cross the fork in the current passage.
    if (!std::all_of(straight.begin(),straight.end(),[&](const Eigen::Vector3f &q) {
          const Eigen::Vector2f d=(q-body).head<2>();
          return std::abs(d.dot(right))<=2.f*radius && d.dot(*heading)>=-radius;
        })) return false;
    route.swap(straight);
    ROS_INFO("Continued measured passage through fork before cached left turn (%.2fm)",advance);
    return true;
  }
  return false;
}

bool FrontierManager::risingBoundaryGuide(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route) {
  vector<Eigen::Vector3f> prefix;
  if (observedBoundaryGuide(body,route,nullptr,nullptr,0.f,&prefix,{},{},true)) return true;
  // An ascent obscured by a bend can still have a measured approach on
  // this level. Re-evaluate the remaining connection as it comes into view.
  route=std::move(prefix);
  return route.size()>1;
}

// For a long revisit near a previous flight, distinguish an opening wide
// enough to explore from isolated missing returns. This scores the proposed
// observation. A transfer waypoint in known air needs no information gain.
bool FrontierManager::hasRemoteOpeningGain(const Eigen::Vector3f &body,const Eigen::Vector3f &stance) const {
  const float trust=frtp_.good_observation_trust_length_;
  if ((stance-body).norm()<=2.f*trust || !nearFlightPath(stance,trust,0.f)) return true;
  const float step=.5f*frtp_.cell_size_,diameter=2.f*graph_->bubble_min_radius_;
  const float clearance=measuredStanceClearance();
  constexpr int bearings=32;
  constexpr float slopes[]={-1.75f,-.8f,-.25f,0.f,.25f,.8f,1.75f};
  unsigned previous=0,first=0;
  for (int i=0;i<bearings;++i) {
    const float yaw=float(2.*M_PI*i/bearings);
    unsigned rays=0;
    for (size_t j=0;j<std::size(slopes);++j) {
      const Eigen::Vector3f ray=Eigen::Vector3f(std::cos(yaw),std::sin(yaw),slopes[j]).normalized();
      float depth=0.f;
      for (float distance=0.f;distance<=2.f*trust;distance+=step) {
        const Eigen::Vector3f p=stance+distance*ray;
        if (!lidar_map_interface_->IsInBox(p) || lidar_map_interface_->getDisToOcc(p)<clearance) break;
        depth=observedAir(p) ? 0.f : depth+step;
        if (depth>=diameter) {rays|=1u<<j;break;}
      }
    }
    const unsigned adjacent=rays&(rays>>1);
    if (previous&adjacent) return true;
    if (i==0) first=adjacent;
    previous=adjacent;
  }
  return (first&previous)!=0;
}

bool FrontierManager::worthwhileRecoveryPass(const Eigen::Vector3f &body,const vector<Eigen::Vector3f> &departure) const {
  // Once the native surface set is spent, compare a remote same-level
  // recovery against the length of the actual unexplored passage. A tiny
  // pocket at the end of a long transfer does not justify flying back.
  const auto &anchor=departure.front();
  const float travel=(anchor-body).norm(),radius=graph_->bubble_min_radius_;
  const float trust=frtp_.good_observation_trust_length_;
  if (!residual_sweep_ || (body-residual_sweep_->first).norm()>radius ||
      travel<2.f*trust || std::abs(anchor.z()-body.z())>radius ||
      std::any_of(departure.begin(),departure.end(),[&](const auto &q){return std::abs(q.z()-anchor.z())>radius;}))
    return true;
  const float required=std::min(trust,2.f*radius+.1f*travel);
  float fresh=0.f;
  const float cell=frtp_.cell_size_;
  for (size_t j=1;j<departure.size();++j) {
    const Eigen::Vector3f edge=departure[j]-departure[j-1];
    const int count=std::max(1,int(std::ceil(edge.norm()/cell)));
    for (int k=1;k<=count;++k) {
      const Eigen::Vector3f q=departure[j-1]+edge*(float(k)/count);
      fresh=nearFlightPath(q,1.5f*radius,0.f,.35f*radius,radius) ? 0.f : fresh+edge.norm()/count;
      if (fresh>=required) return true;
    }
  }
  // A short first guide can expose a genuinely longer passage, for
  // example through a doorway. Verify the extension in measured space.
  ClusterInfo wall;
  if (!measureUnseenWall(departure.back(),wall) || wall.contour_support_.size()*cell<required) return false;
  Eigen::Vector3f tangent=departure.back()-departure[departure.size()-2];
  if (tangent.head<2>().norm()<cell) return false;
  tangent.z()=0.f;tangent.normalize();
  for (float step=cell;fresh<required;step+=cell) {
    const Eigen::Vector3f q=departure.back()+step*tangent;
    if (!certifyFlightGuide({departure.back(),q}) || nearFlightPath(q,1.5f*radius,0.f,.35f*radius,radius))
      return false;
    fresh+=cell;
  }
  return true;
}

// An unvisited opening can be outside the current connected search window.
// Check a different earlier vantage on each exhausted-frontier update. Only
// the measured connector and unflown opening together justify a revisit.
bool FrontierManager::previousOpening(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route) {
  route.clear();
  if (branch_anchors_.empty() || !observedAir(body)) return false;
  surface_body_=body;
  const float clearance=graph_->parallel_bubble_astar_->safe_distance_+.025f;
  const size_t count=branch_anchors_.size();
  const auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(180);
  for (size_t i=0;i<count;++i) {
    Eigen::Vector3f anchor=branch_anchors_[next_branch_anchor_++ % count];
    if ((body-anchor).norm()<frtp_.good_observation_trust_length_ ||
        revisitsLowerPass(body,{anchor})) continue;
    if (!observedAir(anchor) || lidar_map_interface_->getDisToOcc(anchor)<=clearance) {
      // Registration can move the exact historical pose toward a wall.
      // Reobserve the unflown branch from a nearby flown position on the
      // same side. The connector still needs its full cruise clearance.
      const Eigen::Vector3f recorded=anchor;
      float nearest=1.5f*clearance;
      bool found=false;
      for (const auto &p:flight_history_) {
        const float distance=(p-recorded).norm();
        if (distance>=nearest || !observedAir(p) || lidar_map_interface_->getDisToOcc(p)<=clearance) continue;
        vector<Eigen::Vector3f> sight{recorded,p};
        if (!observedGuide(sight) ||
            !graph_->parallel_bubble_astar_->collisionCheck_shortenPath(sight,.25f*clearance)) continue;
        anchor=p;nearest=distance;found=true;
      }
      if (!found) continue;
    }
    vector<Eigen::Vector3f> join,departure,partial,flown_prefix;
    const auto useful=[&](bool found) {
      if (!found || departure.size()<2 || (departure.back()-body).norm()<2.f*clearance ||
          visitedStance(departure.back()) || revisitsLowerPass(body,departure) ||
          !hasRemoteOpeningGain(body,departure.back())) return false;
      return worthwhileRecoveryPass(body,departure);
    };
    // Test the observation, then the next kind of exit from this vantage.
    // A nearby air boundary can be reachable yet reveal nothing from here.
    bool found=useful(observedBoundaryGuide(anchor,departure,nullptr,nullptr,0.f,nullptr,currentSurveyBand(anchor))) ||
               useful(reachableOpening(anchor,departure));
    if (!found && ground_clearance_>0. && std::abs(anchor.z()-body.z())<graph_->bubble_min_radius_) {
      // The local exhaustion cursor belongs to the aircraft. A search from
      // an earlier vantage must not mark that local search complete.
      const auto local_search=ascent_search_;
      const bool local_pending=untested_ascent_;
      found=useful(risingBoundaryGuide(anchor,departure)) &&
            departure.back().z()>anchor.z()+graph_->bubble_min_radius_;
      ascent_search_=local_search;
      untested_ascent_=local_pending;
    }
    surface_body_=body;
    if (!found) {
      if (!residual_sweep_ || next_branch_anchor_>=residual_sweep_->second ||
          std::chrono::steady_clock::now()>=until) return false;
      continue;
    }
    if (!connectObserved(body,anchor,0.,join) && !connectFlightHistory(body,anchor,.035,join,&flown_prefix) &&
        !connectDistantObserved(body,anchor,join,&partial)) {
      if (flown_prefix.size()>1 && (partial.empty() ||
          (flown_prefix.back()-departure.back()).squaredNorm()<(partial.back()-departure.back()).squaredNorm()))
        partial=std::move(flown_prefix);
      if (partial.size()<2 || (departure.back()-partial.back()).norm()+clearance>=(departure.back()-body).norm())
        return false;
      if (revisitsLowerPass(body,partial)) return false;
      route=std::move(partial);
      return true;
    }
    if (!certifyFlightGuide(join)) return false;
    join.insert(join.end(),departure.begin()+1,departure.end());
    if (revisitsLowerPass(body,join) || !certifyFlightGuide(join)) return false;
    route=std::move(join);
    ROS_INFO("Resumed unflown measured opening via prior vantage %.2fm from body",(anchor-body).norm());
    return true;
  }
  return false;
}

bool FrontierManager::initialSurveyGuide(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route) {
  const float radius=graph_->bubble_min_radius_;
  // The first scan can see two parallel walls yet have no sampled stance.
  // Move the sensor within measured free air, far enough for a new baseline.
  // Check all bearings before choosing, independent of initial body yaw.
  const float step=getCellSize();
  const float horizon=std::min(sensorUpdateRange(),6.f*radius);
  float best=2.f*radius;
  Eigen::Vector3f target=body;
  for (int i=0;i<36 && step>0.f;++i) {
    const float angle=i*float(2.*M_PI/36.);
    const Eigen::Vector3f axis(std::cos(angle),std::sin(angle),0.f);
    for (float d=step;d<=horizon;d+=step) {
      const Eigen::Vector3f q=body+d*axis;
      const float clearance=lidar_map_interface_->getDisToOcc(q);
      if (!isInBox(q) || !observedAir(q) || clearance<radius) break;
      const float score=d+.1f*std::min(radius,clearance-radius);
      if (score>best && certifyFlightGuide({body,q})) { best=score; target=q; }
    }
  }
  if ((target-body).norm()<=radius) return false;
  route={body,target};
  ROS_INFO("Issued first measured survey baseline %.2fm",(target-body).norm());
  return true;
}

bool FrontierManager::sightlineGoal(const Eigen::Vector3f &body, vector<Eigen::Vector3f> &route) {
  if (!surface_continuity_ || observed_air_.empty()) return false;
  surface_body_=body;
  const float cell=frtp_.cell_size_;
  const float safe=graph_->bubble_min_radius_;
  const float yaw=graph_->odom_node_->yaw_;
  const double now=epic_ros::Time::now().toSec();
  expireOpeningAttempts(now);
  if (sightline_target_ && now-sightline_selected_at_<8. && (body-*sightline_target_).norm()>cell &&
      supportsElevatedOpening(*sightline_target_) && hasLateralSurveyGain(*sightline_target_)) {
    route={body,*sightline_target_};
    if (certifyFlightGuide(route) ||
        (connectFlightHistory(body,*sightline_target_,.012,route) && certifyFlightGuide(route))) return true;
  }
  sightline_target_.reset();
  // A lateral branch can be visible from the last junction but occluded
  // from the end of the current passage. Reuse a few distinct measured
  // positions and certify the entire path back to the selected opening.
  vector<Eigen::Vector3f> anchors{body};
  for (auto it=flight_history_.rbegin();it!=flight_history_.rend() && anchors.size()<6;++it) {
    if (std::abs(it->z()-body.z())>frtp_.update_length_ || (*it-body).norm()>2.f*frtp_.update_length_) continue;
    if (std::none_of(anchors.begin(),anchors.end(),[&](const auto &other) {
          return (other-*it).norm()<frtp_.good_observation_trust_length_;
        })) anchors.push_back(*it);
  }
  for (const auto &anchor:anchors) {
    vector<Eigen::Vector3f> approach{body};
    if ((anchor-body).norm()>.01f &&
        (!connectFlightHistory(body,anchor,.015,approach) || !certifyFlightGuide(approach))) continue;
    float best_score=0.f;
    Eigen::Vector3f best=anchor;
    // The guide stays in measured air. Unknown cells only contribute evidence
    // of a possible opening beyond its endpoint; no unknown cell is flown into.
    for (int bearing=0;bearing<36;++bearing) {
      const float angle=yaw+bearing*float(M_PI)/18.f;
      const Eigen::Vector3f forward(std::cos(angle),std::sin(angle),0.f);
      for (float range : {2.f*safe, safe, 4.f*safe, 1.75f*safe, .55f*frtp_.update_length_}) {
        for (float dz : {0.f,-cell,cell,2.f*cell,3.f*cell,-3.f*cell}) {
          Eigen::Vector3f goal=anchor+range*forward;
          goal.z()+=dz;
          const Eigen::Vector3i key=(goal/(2.f*cell)).array().floor().cast<int>();
          const bool new_height=std::abs(dz)>safe && !nearFlightPath(goal,.65f*safe,0.f,.2f);
          if (attempted_sightlines_.count(key) || !isInBox(goal) || (visitedStance(goal) && !new_height) ||
              !observedAir(goal) || lidar_map_interface_->getDisToOcc(goal)<safe+.02f ||
              !certifyFlightGuide({anchor,goal})) continue;
          float openings=0.f;
          for (float offset : {-.8f,-.4f,0.f,.4f,.8f}) for (float slope : {-.2f,0.f,.3f}) {
            Eigen::Vector3f ray(std::cos(angle+offset),std::sin(angle+offset),slope);
            ray.normalize();
            for (float r=cell;r<=frtp_.good_observation_trust_length_;r+=cell) {
              const Eigen::Vector3f q=goal+r*ray;
              if (!isInBox(q) || lidar_map_interface_->getDisToOcc(q)<cell) break;
              if (!observedAir(q)) {
                // Translating toward the open end of a long measured ray also
                // exposes new space. Count coherent unknown ahead of the stance
                // even when the entire free prefix is visible from the body.
                const Eigen::Vector3f beyond=q+cell*ray;
                if (isInBox(beyond) && !observedAir(beyond) &&
                    lidar_map_interface_->getDisToOcc(beyond)>cell) {
                  openings+=observedGuide({anchor,q-.5f*cell*ray}) ? .65f : 1.f;
                }
                break;
              }
            }
          }
          const float score=openings*range/(range+safe+std::abs(dz));
          if (score>best_score && supportsElevatedOpening(goal)) {best_score=score;best=goal;}
        }
      }
    }
    if (best_score<=0.f || !hasLateralSurveyGain(best) || !hasRemoteOpeningGain(body,best)) continue;
    recordOpeningAttempt((best/(2.f*cell)).array().floor().cast<int>(),now);
    sightline_target_=best;
    sightline_selected_at_=now;
    route=std::move(approach);route.push_back(best);
    ROS_INFO("Visibility departure, measured guide %.2fm, opening score %.2f",(best-body).norm(),best_score);
    return true;
  }
  return false;
}

void FrontierManager::updateSurfaceSamples() {
  ++surface_scan_;
  const auto &cloud=lidar_map_interface_->ld_->lidar_cloud_.points;
  const Eigen::Vector3f sensor=lidar_map_interface_->ld_->lidar_pose_.cast<float>();
  const float safe=graph_->bubble_min_radius_;
  const float step=.5f*frtp_.cell_size_;
  const size_t stride=std::max<size_t>(1,cloud.size()/1500);
  for (size_t i=surface_scan_%stride;i<cloud.size();i+=stride) {
    const Eigen::Vector3f ray=cloud[i].getVector3fMap()-sensor;
    const float length=ray.norm();
    if (!std::isfinite(length) || length<step) continue;
    const float end=std::min(length-frtp_.cell_size_,frtp_.update_length_);
    for (float at=0.;at<end;at+=step) {
      const Eigen::Vector3f q=sensor+ray*(at/length);
      observed_air_.insert((q/frtp_.cell_size_).array().floor().cast<int>());
    }
  }
  for (size_t i=surface_scan_%3;i<cloud.size();i+=3) {
    const Eigen::Vector3f hit=cloud[i].getVector3fMap();
    if (!hit.allFinite()) continue;
    recordFaceHit(sensor,hit);
    if ((hit-sensor).squaredNorm()<=frtp_.update_length_*frtp_.update_length_ &&
        std::abs(hit.z()-sensor.z())<safe)
      recordMeasuredHit((hit/step).array().floor().cast<int>());
  }
  recordPassage(sensor);
  // The first tour can run several scans after takeoff. Retain a measured
  // enclosure while the vehicle is still acquiring its first survey height.
  if (!perimeter_pass_ && !flight_history_.empty() &&
      (sensor-flight_history_.front()).head<2>().norm()<frtp_.good_observation_trust_length_) {
    const auto level=measuredSurveyHeight(sensor,ground_clearance_);
    if (level && std::abs(*level-sensor.z())<safe) exteriorViewpoint(sensor,true);
  }

}

bool FrontierManager::connectFlightHistory(const Eigen::Vector3f &start,const Eigen::Vector3f &goal,
                                          double budget,vector<Eigen::Vector3f> &route,
                                          vector<Eigen::Vector3f> *prefix) const {
  route.clear();
  if (prefix) prefix->clear();
  if (budget<=0. || !observedAir(start) || !observedAir(goal)) return false;
  // Search a sparse visibility graph in previously flown air. Each edge is
  // rechecked against the current map, including edges between old poses.
  const auto until=std::chrono::steady_clock::now()+std::chrono::duration<double>(budget);
  const size_t begin=flight_history_.size();
  auto point=[&](size_t i)->const Eigen::Vector3f &{
    return i==begin ? start : flight_history_[i];
  };
  using Entry=std::pair<float,size_t>;
  std::priority_queue<Entry,vector<Entry>,std::greater<Entry>> open;
  vector<float> cost(begin+1,std::numeric_limits<float>::infinity());
  vector<size_t> parent(begin+1,begin);
  auto reconstruct=[&](size_t last,vector<Eigen::Vector3f> &path) {
    for (size_t j=last;j!=begin;j=parent[j]) path.push_back(point(j));
    path.push_back(start);
    std::reverse(path.begin(),path.end());
  };
  size_t nearest=begin;
  float nearest_distance=(start-goal).norm();
  vector<Eigen::Vector3f> segment;
  cost[begin]=0.f;open.emplace((start-goal).norm(),begin);
  while (!open.empty() && std::chrono::steady_clock::now()<until) {
    const auto [estimate,i]=open.top();open.pop();
    const auto &from=point(i);
    if (estimate>cost[i]+(from-goal).norm()+1.e-4f) continue;
    // A clear final chord attains the A* lower bound for this settled pose.
    if ((from-goal).norm()<=frtp_.update_length_ && connectObserved(from,goal,0.,segment)) {
      reconstruct(i,route);
      if (route.size()==1 || (route.back()-goal).norm()>1.e-5f) route.push_back(goal);
      return true;
    }
    // Start with recent poses, especially when departing a long dead end.
    for (size_t k=begin;k>0;--k) {
      const size_t j=k-1;
      if (std::chrono::steady_clock::now()>=until) break;
      const float distance=(point(j)-from).norm();
      const float next=cost[i]+distance;
      if (distance>frtp_.update_length_ || next>=cost[j] ||
          !connectObserved(from,point(j),0.,segment)) continue;
      parent[j]=i;cost[j]=next;
      const float remaining=(point(j)-goal).norm();
      if (prefix && remaining<nearest_distance) {nearest=j;nearest_distance=remaining;}
      open.emplace(next+remaining,j);
    }
  }
  // The remote observation is still pending. A checked prefix can move the
  // vehicle within range of its next search without committing the unseen gap.
  if (prefix && nearest_distance+2.f*graph_->bubble_min_radius_<(start-goal).norm()) {
    reconstruct(nearest,*prefix);
    if (!certifyFlightGuide(*prefix)) prefix->clear();
  }
  return false;
}

// A single clearance maximum per cross-section can end at a protrusion.
// Keep reachable heights in each section until the following section shows
// which of them connects to the exact observation.
bool FrontierManager::connectObservedSections(const Eigen::Vector3f &start,const Eigen::Vector3f &goal,
                                             vector<Eigen::Vector3f> &route,bool aperture) const {
  const float cell=frtp_.cell_size_,cruise=aperture?aperture_clearance_:graph_->parallel_bubble_astar_->safe_distance_;
  const Eigen::Vector3f leg=goal-start;
  const float length=leg.norm();
  if (length<=cell || length>frtp_.update_length_ || !observedAir(goal)) return false;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(160);
  const Eigen::Vector3f axis=leg/length;
  Eigen::Vector3f side=axis.cross(Eigen::Vector3f::UnitZ());
  if (side.squaredNorm()<.01f) side=axis.unitOrthogonal();
  side.normalize();
  const Eigen::Vector3f up=side.cross(axis);
  const int sections=int(std::ceil(length/(.25f*cell)));
  struct Stance {Eigen::Vector3f point;float cost;size_t parent;};
  vector<vector<Stance>> layers{{{start,0.f,0}}};
  const auto clearance=[&](const Eigen::Vector3f &q) -> float {
    if (!observedAir(q) || !lidar_map_interface_->IsInBox(q) || q.z()<graph_->parallel_bubble_astar_->flight_min_z_ ||
        q.z()>graph_->parallel_bubble_astar_->flight_max_z_) return 0.f;
    return lidar_map_interface_->getDisToOcc(q);
  };
  for (int i=1;i<=sections;++i) {
    if (std::chrono::steady_clock::now()>=deadline) return false;
    const Eigen::Vector3f centre=start+leg*(float(i)/sections);
    const auto &previous=layers.back();
    vector<Stance> current;
    const int heights=i==sections ? 0:15;
    for (int h=-heights;h<=heights;++h) {
      const Eigen::Vector3f base=centre+float(h)*(.125f*cell)*up;
      Eigen::Vector3f best=base;
      float margin=clearance(best);
      const auto consider=[&](const Eigen::Vector3f &q) {
        const float d=clearance(q);
        if (d>margin) {margin=d;best=q;}
      };
      if (i<sections) {
        for (int j=-6;j<=6;++j) consider(base+float(j)*(.35f*cell)*side);
        // Refine an observed clearance peak within half a sample spacing
        // of the clearance required to cross this section.
        if (margin+.175f*cell<cruise && margin>0.f) continue;
        for (float step=.175f*cell;step>=.01f*cell;step*=.5f) {
          const Eigen::Vector3f last=best;
          consider(last+step*side);
          consider(last-step*side);
        }
      }
      if (margin<=cruise+.001f) continue;
      vector<std::pair<float,size_t>> parents;
      for (size_t j=0;j<previous.size();++j) {
        const float distance=(best-previous[j].point).norm();
        if (distance>.68f*cell) continue;
        const float cost=previous[j].cost+distance*(1.f+.01f/(.01f+margin-cruise));
        parents.emplace_back(cost,j);
      }
      std::sort(parents.begin(),parents.end());
      vector<Eigen::Vector3f> edge;
      for (const auto &[cost,j]:parents) {
        edge={previous[j].point,best};
        if (!observedGuide(edge) || !graph_->parallel_bubble_astar_->collisionCheck_shortenPath(edge,cruise)) continue;
        current.push_back({best,cost,j});
        break;
      }
    }
    if (current.empty()) return false;
    layers.push_back(std::move(current));
  }
  vector<Eigen::Vector3f> path(layers.size());
  size_t j=0;
  for (size_t i=layers.size();i-->0;) {
    path[i]=layers[i][j].point;
    j=layers[i][j].parent;
  }
  if (!(aperture?certifyShortAperture(path):certifyFlightGuide(path))) return false;
  route.swap(path);
  return true;
}

// Admit an already sampled observation to the tour when only a short, fully
// measured restriction interrupts an otherwise progressing horizontal pass.
void FrontierManager::connectForwardAperture(const Eigen::Vector3f &body,const vector<TopoNode::Ptr> &views) {
  if (aperture_clearance_<=0. || !surface_continuity_ || views.empty()) return;
  const auto heading=surveyFlightDirection(body);
  if (!heading) return;
  const float radius=graph_->bubble_min_radius_;
  vector<std::pair<float,TopoNode::Ptr>> candidates;
  for (const auto &view:views) {
    const auto it=observations_.find(view.get());
    if (it==observations_.end() || !it->second) continue;
    const Eigen::Vector3f offset=view->center_-body;
    const float ahead=offset.head<2>().dot(*heading);
    if (ahead<2.5f*radius || ahead<.85f*offset.head<2>().norm() ||
        std::abs(offset.z())>1.5f*radius || offset.norm()>2.f*frtp_.good_observation_trust_length_ ||
        visitedStance(view->center_) || !observedAir(view->center_) ||
        lidar_map_interface_->getDisToOcc(view->center_)<measuredStanceClearance()) continue;
    if (certifyCruiseGuide({body,view->center_}) ||
        (it->second->contour_approach_.size()>1 &&
         (it->second->contour_approach_.front()-body).norm()<radius &&
         certifyCruiseGuide(it->second->contour_approach_))) return;
    candidates.emplace_back(offset.norm(),view);
  }
  std::stable_sort(candidates.begin(),candidates.end(),[](const auto &a,const auto &b){return a.first<b.first;});
  for (size_t i=0;i<std::min<size_t>(2,candidates.size());++i) {
    const auto &view=candidates[i].second;
    vector<Eigen::Vector3f> path;
    if (!connectObservedSections(body,view->center_,path,true)) continue;
    auto &obs=*observations_.at(view.get());
    obs.contour_approach_=std::move(path);
    obs.aperture_approach_=true;
    last_aperture_path_=obs.contour_approach_;
    ROS_INFO("Connected forward observation through measured aperture at %.2f %.2f %.2f",
                      view->center_.x(),view->center_.y(),view->center_.z());
    return;
  }
}

// The vehicle moves between the global solve and the handoff. Reconnect
// the same selected observation from fresh odometry on the latest scan.
bool FrontierManager::reconnectSelectedApproach(const Eigen::Vector3f &body,const Eigen::Vector3f &goal,
                                               vector<Eigen::Vector3f> &route) {
  if (!surface_continuity_ || !withinFlightHeight(body) || !withinFlightHeight(goal)) return false;
  surface_body_=body;
  vector<Eigen::Vector3f> path{body,goal};
  if (!certifyFlightGuide(path) &&
      !(connectObserved(body,goal,.025,path) && certifyFlightGuide(path))) {
    // The update radius bounds a new aperture search. Air accumulated along a
    // longer, already observed route was checked above at normal clearance.
    if ((goal-body).norm()>frtp_.update_length_) return false;
    const bool known_crossing=last_aperture_path_.size()>1 && (last_aperture_path_.back()-goal).norm()<=.05f;
    // A stopped aircraft may have lost its direction estimate on a return
    // leg. The selected stance and the measured flight on both sides still
    // identify the crossing. Recheck every segment against the current scan.
    const bool flown_crossing=aperture_clearance_>0. && std::abs(goal.z()-body.z())<graph_->bubble_min_radius_ &&
        (goal-body).norm()<=2.f*frtp_.good_observation_trust_length_ &&
        flownObservationStance(body) && flownObservationStance(goal);
    if ((!known_crossing && !flown_crossing) || !connectObservedSections(body,goal,path,true)) return false;
    last_aperture_path_=path;
  }
  route.swap(path);
  return true;
}

// Recovery may need to cross the same restriction on its way back to a
// pending branch. Use actual earlier positions on the continuing flight line.
bool FrontierManager::resumeSurveyThroughAperture(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route) {
  if (aperture_clearance_<=0. || branch_anchors_.empty() || !flownObservationStance(body)) return false;
  const auto course=surveyFlightDirection(body);
  if (!course) return false;
  surface_body_=body;
  const float radius=graph_->bubble_min_radius_,trust=frtp_.good_observation_trust_length_;
  vector<std::pair<float,Eigen::Vector3f>> stances;
  const auto consider=[&](const Eigen::Vector3f &point) {
    const Eigen::Vector3f delta=point-body;
    const float ahead=delta.head<2>().dot(*course);
    if (ahead<3.f*radius || ahead>2.f*trust || std::abs(delta.z())>radius ||
        (delta.head<2>()-ahead*(*course)).norm()>2.f*radius ||
        !observedAir(point) || !flownObservationStance(point)) return;
    const float space=lidar_map_interface_->getDisToOcc(point);
    if (space<measuredStanceClearance()) return;
    stances.emplace_back(std::abs(ahead-1.35f*trust)-std::min(radius,space),point);
  };
  for (const auto &point:flight_history_) consider(point);
  for (const auto &point:last_aperture_path_) consider(point);
  std::stable_sort(stances.begin(),stances.end(),[](const auto &a,const auto &b){return a.first<b.first;});
  vector<Eigen::Vector3f> tried;
  for (const auto &candidate:stances) {
    const auto &point=candidate.second;
    if (std::any_of(tried.begin(),tried.end(),[&](const auto &p){return (p-point).norm()<radius;})) continue;
    if (tried.size()>=4) break;
    tried.push_back(point);
    vector<Eigen::Vector3f> path;
    if (certifyCruiseGuide({body,point}) || !connectObservedSections(body,point,path,true) ||
        certifyCruiseGuide(path)) continue;
    last_aperture_path_=path;
    route=std::move(path);
    ROS_INFO("Resumed measured return through aperture from %.2f %.2f to %.2f %.2f",
             body.x(),body.y(),point.x(),point.y());
    return true;
  }
  return false;
}

bool FrontierManager::connectDistantObserved(const Eigen::Vector3f &start,const Eigen::Vector3f &goal,
                                           vector<Eigen::Vector3f> &route,vector<Eigen::Vector3f> *prefix) const {
  if (prefix) prefix->clear();
  const float cell=frtp_.cell_size_,safe=graph_->parallel_bubble_astar_->safe_distance_;
  if (connectObservedGrid(start,goal,cell,.14,route,true) && certifyFlightGuide(route)) return true;
  route.clear();
  // Coarse voxel centres can miss a traversable band between them. Use an
  // optimistic search only to locate that band. Nothing from this search
  // is published until fine-grid connections pass full flight certification.
  const auto index=[&](const Eigen::Vector3f &q)->Eigen::Vector3i {
    return (q/cell).array().floor().cast<int>();
  };
  const auto centre=[&](const Eigen::Vector3i &k)->Eigen::Vector3f {
    return (k.cast<float>().array()+.5f)*cell;
  };
  const auto first=index(start),last=index(goal);
  struct Entry {Eigen::Vector3i key;float cost,estimate;};
  const auto later=[](const Entry &a,const Entry &b){return a.estimate>b.estimate;};
  std::priority_queue<Entry,vector<Entry>,decltype(later)> open(later);
  std::unordered_map<Eigen::Vector3i,float,Vector3i_Hash> cost,space;
  std::unordered_map<Eigen::Vector3i,Eigen::Vector3i,Vector3i_Hash> parent;
  cost[first]=0.f;open.push({first,0.f,(goal-start).norm()});
  vector<Eigen::Vector3f> spine;
  const auto begin=std::chrono::steady_clock::now();
  while (!open.empty() && std::chrono::steady_clock::now()-begin<std::chrono::milliseconds(250)) {
    const auto top=open.top();open.pop();
    if (top.cost>cost.at(top.key)) continue;
    if (top.key==last) {
      spine.push_back(goal);
      for (auto k=last;k!=first;) {k=parent.at(k);if(k!=first) spine.push_back(centre(k));}
      spine.push_back(start);std::reverse(spine.begin(),spine.end());break;
    }
    for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y) for (int z=-1;z<=1;++z) {
      if (!x && !y && !z) continue;
      const Eigen::Vector3i delta(x,y,z),key=top.key+delta;
      if (!observed_air_.count(key)) continue;
      const auto q=centre(key);
      auto [it,inserted]=space.try_emplace(key,0.f);
      if (inserted) it->second=lidar_map_interface_->getDisToOcc(q);
      if (it->second+cell*.7f<safe) continue;
      const float next=top.cost+delta.cast<float>().norm()*cell+
          4.f*std::max(0.f,safe+.25f*cell-it->second);
      const auto old=cost.find(key);
      if (old!=cost.end() && old->second<=next) continue;
      cost[key]=next;parent[key]=top.key;open.push({key,next,next+1.4f*(q-goal).norm()});
    }
  }
  const bool trace=std::getenv("EPIC_TRACE_CONNECTOR")!=nullptr;
  if (trace) ROS_INFO("Measured connector spine %zu cells; departure %.2f %.2f %.2f, goal %.2f %.2f %.2f",
      spine.size(),start.x(),start.y(),start.z(),goal.x(),goal.y(),goal.z());
  if (spine.size()>1) {
    // Refine each cross-section of the measured route before connecting its
    // vertices. A clearance ridge may be only a few centimetres wide, even
    // though adjacent sections belong to one connected free-space passage.
    route={start};
    vector<Eigen::Vector3f> edge;
    bool connected=true;
    for (size_t j=1;j<spine.size() && connected;++j) {
      const auto &nominal=spine[j];
      if (lidar_map_interface_->getDisToOcc(nominal)>safe+.3f*cell &&
          connectObserved(route.back(),nominal,0.,edge)) {
        route.push_back(nominal);
        continue;
      }
      Eigen::Vector3f tangent=spine[std::min(spine.size()-1,j+2)]-spine[j>2 ? j-2 : 0];
      if (tangent.norm()<.001f) break;
      tangent.normalize();
      const Eigen::Vector3f side=tangent.unitOrthogonal(),up=tangent.cross(side);
      const Eigen::Vector3f segment=nominal-spine[j-1];
      const int divisions=std::max(1,int(std::ceil(segment.norm()/(.25f*cell))));
      for (int k=1;k<=divisions;++k) {
        const Eigen::Vector3f centre=spine[j-1]+segment*(float(k)/divisions);
        const Eigen::Vector3f offset=route.back()+segment/divisions-centre;
        const Eigen::Vector3f predicted=centre+offset-tangent*offset.dot(tangent);
        Eigen::Vector3f q=centre;
        if (j+1<spine.size() || k<divisions) {
          float best=-std::numeric_limits<float>::infinity();
          const auto consider=[&](const Eigen::Vector3f &probe) {
            if ((probe-centre).norm()>2.f*cell || !observedAir(probe) || !lidar_map_interface_->IsInBox(probe) ||
                probe.z()<graph_->parallel_bubble_astar_->flight_min_z_ ||
                probe.z()>graph_->parallel_bubble_astar_->flight_max_z_) return;
            const float score=lidar_map_interface_->getDisToOcc(probe)-
                .025f*(probe-centre).norm()-.06f*(probe-predicted).norm();
            if (score>best) {best=score;q=probe;}
          };
          consider(predicted);
          for (int x=-3;x<=3;++x) for (int y=-3;y<=3;++y)
            consider(centre+.4f*cell*(x*side+y*up));
          for (const float fraction:{.2f,.1f,.05f,.025f}) {
            const Eigen::Vector3f base=q;
            for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y)
              consider(base+fraction*cell*(x*side+y*up));
          }
        }
        if (!connectObserved(route.back(),q,0.,edge)) {
          if (trace) ROS_INFO("Measured ridge interrupted at cell %zu, last %.2f %.2f %.2f, target %.2f %.2f %.2f",
              j,route.back().x(),route.back().y(),route.back().z(),q.x(),q.y(),q.z());
          connected=false;break;
        }
        route.push_back(q);
      }
    }
    if ((route.back()-goal).norm()<.005f && certifyFlightGuide(route)) return true;
    if (route.size()>1 && connectObservedSections(start,goal,route)) return true;
    // A verified prefix can advance a return while a further connection is
    // being searched. Callers joining two observations still require the
    // entire connector and keep the exact endpoint.
    if (prefix && route.size()>1 && (route.back()-start).norm()>3.f*safe &&
        (route.back()-goal).norm()<(start-goal).norm()-safe && certifyFlightGuide(route)) *prefix=route;
    route={start};
    for (size_t i=1;i<spine.size();) {
      size_t end=i;
      while (end+1<spine.size() && (spine[end]-spine[i-1]).norm()<6.f*cell) ++end;
      const bool endpoint=end+1==spine.size();
      vector<Eigen::Vector3f> targets{spine[end]};
      if (!endpoint) {
        for (int x=-2;x<=2;++x) for (int y=-2;y<=2;++y) for (int z=-2;z<=2;++z) {
          if (!x && !y && !z) continue;
          const auto q=(spine[end]+.5f*cell*Eigen::Vector3f(x,y,z)).eval();
          if (observedAir(q) && lidar_map_interface_->getDisToOcc(q)>safe+.005f) targets.push_back(q);
        }
        std::sort(targets.begin(),targets.end(),[&](const auto &a,const auto &b){
          return (a-spine[end]).squaredNorm()< (b-spine[end]).squaredNorm();
        });
      }
      vector<Eigen::Vector3f> leg;
      bool linked=false;
      for (const auto &q:targets) {
        if (connectObserved(route.back(),q,0.,leg)) {linked=true;break;}
      }
      if (!linked) {
        size_t attempts=0;
        for (const auto &q:targets) {
          if (lidar_map_interface_->getDisToOcc(q)<safe+.01f) continue;
          if (connectObservedGrid(route.back(),q,.5f*cell,.06,leg,true) ||
              connectObservedGrid(route.back(),q,cell/3.f,.08,leg,true) ||
              connectObservedGrid(route.back(),q,cell/8.f,.08,leg,true)) {linked=true;break;}
          if (++attempts==3 || std::chrono::steady_clock::now()-begin>std::chrono::milliseconds(950)) break;
        }
      }
      if (!linked) {
        if (trace) ROS_INFO("Measured connector stalled at cell %zu/%zu with %zu endpoints",end,spine.size(),targets.size());
        break;
      }
      route.insert(route.end(),leg.begin()+1,leg.end());
      if (endpoint && certifyFlightGuide(route)) return true;
      i=end+1;
    }
  }
  route.clear();
  return false;
}


// Repeated views from the same line cannot establish that the surrounding
// volume is inspected. Require known, unobstructed sight from poses that
// enclose the stance in the horizontal plane at its own sensor height.
bool FrontierManager::surveyedFromAround(const Eigen::Vector3f &body,const Eigen::Vector3f &point) const {
  const float radius=graph_->bubble_min_radius_;
  const float range=std::min(frtp_.update_length_,2.f*frtp_.good_observation_trust_length_);
  if ((body-point).norm()<=range || !observedAir(point)) return false;
  constexpr int directions=16;
  std::array<bool,directions> seen{};
  for (auto it=flight_history_.rbegin();it!=flight_history_.rend();++it) {
    const Eigen::Vector3f offset=*it-point;
    const float range_xy=offset.head<2>().squaredNorm();
    if (std::abs(offset.z())>radius || range_xy<radius*radius || range_xy>range*range) continue;
    const float angle=std::atan2(offset.y(),offset.x());
    const int k=static_cast<int>((angle+M_PI)*(directions/(2.*M_PI)))%directions;
    if (seen[k]) continue;
    vector<Eigen::Vector3f> ray{*it,point};
    if (!observedGuide(ray) || !graph_->parallel_bubble_astar_->collisionCheck_shortenPath(ray,.5f*frtp_.cell_size_)) continue;
    seen[k]=true;
    int gap=0,longest=0;
    for (int j=1;j<=directions;++j) {
      gap=seen[(k+j)%directions] ? 0:gap+1;
      longest=std::max(longest,gap);
    }
    if (longest<=5) return true;
  }
  return false;
}

bool FrontierManager::nearFlightPath(const Eigen::Vector3f &point,float radius,float ignore_recent,
                                     float sight_clearance,float height_tolerance) const {
  size_t end=flight_history_.size();
  while (end>1 && ignore_recent>0.f) {
    ignore_recent-=(flight_history_[end-1]-flight_history_[end-2]).norm();
    --end;
  }
  for (size_t i=1;i<end;++i) {
    const auto &a=flight_history_[i-1],&b=flight_history_[i];
    const Eigen::Vector3f delta=b-a;
    const float t=std::clamp((point-a).dot(delta)/std::max(.001f,delta.squaredNorm()),0.f,1.f);
    const Eigen::Vector3f closest=a+t*delta;
    if (std::abs(point.z()-closest.z())>height_tolerance || (point-closest).norm()>=radius) continue;
    vector<Eigen::Vector3f> sight{point,closest};
    if (observedGuide(sight) && (sight_clearance<=0.f ||
        graph_->parallel_bubble_astar_->collisionCheck_shortenPath(sight,sight_clearance))) return true;
  }
  return false;
}

// Compare candidate stances against older actual flight on the same side of a
// wall. Excluding the most recent leg leaves the rolling forward guide usable.
bool FrontierManager::flownObservationStance(const Eigen::Vector3f &point) const {
  const float radius=graph_->bubble_min_radius_,trust=frtp_.good_observation_trust_length_;
  return nearFlightPath(point,std::min(3.5f*radius,.65f*trust),2.f*trust,.35f*radius,radius);
}

bool FrontierManager::surveyedEnclosureStance(const Eigen::Vector3f &point) const {
  return perimeter_pass_ && perimeter_pass_->finished &&
      std::abs(point.z()-perimeter_pass_->level)<=graph_->bubble_min_radius_ && flownObservationStance(point);
}

bool FrontierManager::spentSurveyObservation(const ClusterInfo &observation,const Eigen::Vector3f &stance) const {
  if (!surveyedEnclosureStance(stance)) {
    if (!surface_body_ || observation.exterior_pass_ || !flownObservationStance(stance)) return false;
    const float radius=graph_->bubble_min_radius_,trust=frtp_.good_observation_trust_length_;
    const Eigen::Vector3f delta=stance-*surface_body_;
    const float length=delta.head<2>().norm();
    if (length<=2.f*trust || std::abs(delta.z())>radius) return false;
    // A remote stance on a passage flown in both directions needs fresh
    // wall evidence. A bend, an unflown side opening or a rise stays eligible.
    const Eigen::Vector2f axis=delta.head<2>()/length;
    bool forward=false,back=false;
    for (size_t i=1;i<flight_history_.size() && !(forward && back);++i) {
      const auto &a=flight_history_[i-1],&b=flight_history_[i];
      const Eigen::Vector3f edge=b-a;
      const float span=edge.head<2>().norm();
      if (span<.15f) continue;
      const float t=std::clamp((stance-a).dot(edge)/edge.squaredNorm(),0.f,1.f);
      const Eigen::Vector3f nearest=a+t*edge;
      if ((nearest-stance).norm()>1.5f*radius) continue;
      vector<Eigen::Vector3f> sight{stance,nearest};
      if (!observedGuide(sight) || !graph_->parallel_bubble_astar_->collisionCheck_shortenPath(sight,.2f)) continue;
      const float alignment=edge.head<2>().dot(axis)/span;
      forward|=alignment>.8f;
      back|=alignment<-.8f;
    }
    if (!forward || !back) return false;
    const int samples=static_cast<int>(std::ceil(length/(2.f*radius)));
    for (int j=1;j<samples;++j) {
      const Eigen::Vector3f q=*surface_body_+delta*(float(j)/samples);
      if (!nearFlightPath(q,2.f*radius,0.f,0.f,radius)) return false;
    }
  }
  // A second flight along the same visible aisle needs a coherent new wall
  // patch. Lateral registration differences and sparse voxel edges are common.
  const float radius=graph_->bubble_min_radius_,cell=frtp_.cell_size_;
  auto support=unseenSupport(observation);
  std::unordered_set<Eigen::Vector3i,Vector3i_Hash> cells(support.begin(),support.end());
  if (cells.size()*cell*cell>=radius*radius && hasRevisitableSurface(observation,stance)) return false;
  // An observed connector may lead through the old aisle to an unflown
  // opening. Preserve a measured preview that actually leaves that aisle.
  const auto &preview=observation.contour_preview_;
  return preview.size()<2 || (preview.back()-stance).norm()<frtp_.good_observation_trust_length_ ||
      flownObservationStance(preview.back()) || !hasLateralSurveyGain(preview.back());
}

bool FrontierManager::needsOppositeTransit(const Eigen::Vector3f &point,const Eigen::Vector2f &direction) const {
  const float safe=graph_->bubble_min_radius_;
  bool crossed=false;
  float recent=0.f;
  for (size_t i=flight_history_.size();i>1;--i) {
    const auto &a=flight_history_[i-2],&b=flight_history_[i-1];
    const Eigen::Vector3f edge=b-a;
    recent+=edge.norm();
    if (recent<4.f*safe || edge.head<2>().norm()<.15f) continue;
    const float t=std::clamp((point-a).dot(edge)/edge.squaredNorm(),0.f,1.f);
    const Eigen::Vector3f nearest=a+t*edge;
    if ((point-nearest).norm()>1.5f*safe) continue;
    vector<Eigen::Vector3f> sight{point,nearest};
    if (!observedGuide(sight) || !graph_->parallel_bubble_astar_->collisionCheck_shortenPath(sight,.2f)) continue;
    const float alignment=edge.head<2>().normalized().dot(direction);
    if (alignment>.55f) return false;
    if (alignment<.15f) crossed=true;
  }
  return crossed;
}

void FrontierManager::recordPassage(const Eigen::Vector3f &body) {
  const float safe=graph_->bubble_min_radius_;
  if (flight_history_.empty()) flight_history_.push_back(body);
  const Eigen::Vector3f travelled=body-flight_history_.back();
  if (travelled.norm()>.35f) {
    const int steps=std::max(1,int(std::ceil(travelled.norm()/frtp_.cell_size_)));
    const bool retracing=nearFlightPath(body,1.5f*safe,4.f*safe);
    // A few centimetres outside the earlier track do not erase an entire
    // repeated pass. Sustained motion into fresh air clears the history.
    retraced_distance_=std::max(0.f,retraced_distance_+(retracing ? 1.f : -2.f)*travelled.norm());
    const float voxel=frtp_.cell_size_;
    const int radius=int(std::ceil(1.4f*safe/voxel));
    for (int k=1;k<=steps;++k) {
      const Eigen::Vector3f p=flight_history_.back()+travelled*(float(k)/steps);
      const Eigen::Vector3i base=(p/voxel).array().floor().cast<int>();
      for (int x=-radius;x<=radius;++x) for (int y=-radius;y<=radius;++y) for (int z=-radius;z<=radius;++z) {
        const Eigen::Vector3i key=base+Eigen::Vector3i(x,y,z);
        if (visited_air_.count(key) || !observed_air_.count(key)) continue;
        const Eigen::Vector3f q=(key.cast<float>().array()+.5f)*voxel;
        if ((q-p).norm()>1.4f*safe || !observedGuide({p,q})) continue;
        vector<Eigen::Vector3f> sight{p,q};
        if (graph_->parallel_bubble_astar_->collisionCheck_shortenPath(sight,.2f)) visited_air_.insert(key);
      }
    }
    if (ground_clearance_>0.) {
      // Confirm a level by actual horizontal travel. A single high pose
      // during a turn or a climb cannot move the remembered survey height.
      float lo=body.z(),hi=lo;
      for (auto it=flight_history_.rbegin();it!=flight_history_.rend();++it) {
        hi=std::max(hi,it->z());lo=std::min(lo,it->z());
        if (hi-lo>frtp_.cell_size_) break;
        if ((body-*it).head<2>().norm()>=2.f*safe) {
          const float level=.5f*(lo+hi);
          highest_flown_band_=std::max(highest_flown_band_.value_or(level),level);
          lowest_flown_band_=std::min(lowest_flown_band_.value_or(level),level);
          break;
        }
      }
    }
    flight_history_.push_back(body);
    // Sparse flight poses retain access to branches once the current scan
    // can no longer see their entrance. They do not become viewpoints.
    const float spacing=std::max(3.f*safe,frtp_.good_observation_trust_length_);
    if ((branch_anchors_.empty() || (body-branch_anchors_.back()).norm()>=spacing) &&
        std::none_of(branch_anchors_.begin(),branch_anchors_.end(),[&](const auto &p) {
          return (body-p).squaredNorm()<spacing*spacing;
        })) branch_anchors_.push_back(body);
  }
}

// A short braking or tracking reversal does not change the direction of a
// sustained survey pass. Use distance along the physically flown trace.
std::optional<Eigen::Vector2f> FrontierManager::surveyFlightDirection(const Eigen::Vector3f &body) const {
  const float radius=graph_->bubble_min_radius_;
  const float span=std::max(6.f*radius,2.f*frtp_.good_observation_trust_length_);
  float arc=0.f;
  Eigen::Vector3f previous=body;
  for (auto it=flight_history_.rbegin();it!=flight_history_.rend();++it) {
    if (std::abs(it->z()-body.z())>1.5f*radius) return {};
    arc+=(*it-previous).norm();
    previous=*it;
    if (arc<span) continue;
    const Eigen::Vector2f chord=(body-*it).head<2>();
    if (chord.norm()<.65f*arc) return {};
    return chord.normalized();
  }
  return {};
}

// Find broad support below the vehicle. A single vertical face or narrow shelf
// edge cannot establish a floor. Bin relative to the map resolution so a
// sloping or noisy surface can occupy neighbouring height slices.
std::optional<float> FrontierManager::measuredSurveyHeight(const Eigen::Vector3f &body,float clearance) const {
  if (clearance<=0.f) return std::nullopt;
  const float radius=std::max<float>(graph_->bubble_min_radius_,.9f*clearance);
  const float depth=std::min(frtp_.update_length_,5.f*clearance);
  const float height_bin=.6f*frtp_.cell_size_;
  const float min_drop=.75f*graph_->bubble_min_radius_;
  if (depth<=min_drop || height_bin<=0.f) return std::nullopt;
  PointVector hits;
  lidar_map_interface_->boxSearch(body-Eigen::Vector3f(radius,radius,depth),
      body+Eigen::Vector3f(radius,radius,-min_drop),hits);
  std::vector<unsigned> sectors(static_cast<size_t>(std::ceil(depth/height_bin))+2,0);
  std::vector<unsigned> central_sectors(sectors.size(),0);
  std::vector<vector<float>> central_heights(sectors.size());
  for (const auto &hit:hits) {
    if (!std::isfinite(hit.x) || !std::isfinite(hit.y) || !std::isfinite(hit.z)) continue;
    const Eigen::Vector2f offset=hit.getVector3fMap().head<2>()-body.head<2>();
    if (offset.squaredNorm()>radius*radius || offset.squaredNorm()<.04f*radius*radius) continue;
    const int band=static_cast<int>(std::floor((body.z()-hit.z)/height_bin));
    if (band<0 || size_t(band)>=sectors.size()) continue;
    const unsigned octant=static_cast<unsigned>((std::atan2(offset.y(),offset.x())+M_PI)*4./M_PI)%8;
    sectors[band]|=1u<<octant;
    if (offset.squaredNorm()<.55f*.55f*radius*radius) {
      central_sectors[band]|=1u<<octant;
      central_heights[band].push_back(hit.z);
    }
  }
  for (size_t i=1;i+1<sectors.size();++i) {
    if (std::bitset<8>(sectors[i]).count()<3 ||
        std::bitset<8>(sectors[i-1]|sectors[i]|sectors[i+1]).count()<6) continue;
    // Two upright surfaces can jointly occupy nearly every outer sector.
    // A supporting surface also spans the space directly under the body.
    const unsigned inner=central_sectors[i-1]|central_sectors[i]|central_sectors[i+1];
    if (std::bitset<8>(inner).count()<4 || !(inner & (inner>>4))) continue;
    vector<float> heights;
    for (size_t band=i-1;band<=i+1;++band)
      heights.insert(heights.end(),central_heights[band].begin(),central_heights[band].end());
    auto middle=heights.begin()+heights.size()/2;
    std::nth_element(heights.begin(),middle,heights.end());
    return *middle+clearance;
  }
  return std::nullopt;
}

// Signed area distinguishes a right-side perimeter circuit from a return
// down a passage. The short final gap must also be visible and traversable.
bool FrontierManager::closesBoundarySweep(const vector<Eigen::Vector3f> &route) const {
  if (route.size()<3 || flight_history_.size()<3) return false;
  const float radius=graph_->bubble_min_radius_;
  const auto &body=route.front(), &tip=route.back();
  // Crossing a just-used approach already completed the local circuit.
  // A different wall beyond that crossing belongs to the next choice.
  if (nearFlightPath(body,1.8f*radius,9.f*radius,.5f*radius)) return false;
  const auto cross=[&](const Eigen::Vector3f &a,const Eigen::Vector3f &b) {
    return (a.x()-body.x())*(b.y()-body.y())-(a.y()-body.y())*(b.x()-body.x());
  };
  float twice_area=0.f,planned=0.f;
  for (size_t i=1;i<route.size();++i) {
    twice_area+=cross(route[i-1],route[i]);
    planned+=(route[i]-route[i-1]).norm();
  }
  float flown=0.f;
  Eigen::Vector3f next=body;
  for (auto it=flight_history_.rbegin();it!=flight_history_.rend();++it) {
    if (std::abs(it->z()-body.z())>2.f*radius) break;
    flown+=(*it-next).norm();
    if (flown>4.f*frtp_.update_length_) break;
    twice_area+=cross(*it,next);next=*it;
    const float gap=(tip-*it).norm();
    if (flown<12.f*radius || gap>5.f*radius || gap>.55f*planned) continue;
    const float perimeter=flown+planned+gap;
    // 4 pi A / P² is dimensionless. A line doubled back has zero area.
    if (2.f*float(M_PI)*(twice_area+cross(tip,*it))<.5f*perimeter*perimeter) continue;
    if (gap<.05f || certifyFlightGuide({tip,*it})) return true;
  }
  return false;
}

void FrontierManager::appendContourViewpoints(const Eigen::Vector3f &body, vector<TopoNode::Ptr> &views,bool include_alternative) {
  // Prefer a wider wall offset in free flight. A certified connector may
  // still use the topology clearance where the measured passage narrows.
  double preferred_clearance;
  nh_.param("FrontierManager/preferred_wall_clearance",preferred_clearance,graph_->bubble_min_radius_);
  const float safe=std::max(graph_->bubble_min_radius_,preferred_clearance);
  const float minimum_clearance=measuredStanceClearance();
  const float cell=.5f*frtp_.cell_size_;
  surface_body_=body;
  perimeter_target_.reset();
  if (flight_history_.empty()) flight_history_.push_back(body);
  // Avoid drawing a vehicle that has climbed back onto its launch plane.
  // Small height errors on level ground still share one stable scan slice.
  float z=std::abs(body.z()-vpp_.fixed_height_)<frtp_.cell_size_ ? vpp_.fixed_height_ : body.z();
  survey_height_=measuredSurveyHeight(body,float(ground_clearance_));
  if (survey_height_)
    z=body.z()+std::clamp(*survey_height_-body.z(),-frtp_.cell_size_,frtp_.cell_size_);
  const int half=int(std::ceil((frtp_.update_length_+4.f*safe)/cell));
  const Eigen::Vector2i base=(body.head<2>()/cell).array().floor().cast<int>();
  cv::Mat below(2*half+1,2*half+1,CV_8U,cv::Scalar(0)),above=below.clone();
  for (const auto &[layer, keys] : measured_hits_by_layer_) {
    const float dz=(layer+.5f)*cell-z;
    if (std::abs(dz)>=.6f*safe) continue;
    for (const auto &key : keys) {
      const int x=key.x()-base.x()+half,y=key.y()-base.y()+half;
      if (x<0 || x>=below.cols || y<0 || y>=below.rows) continue;
      if (dz<-.4f*cell) below.at<uchar>(y,x)=255;
      if (dz>.4f*cell) above.at<uchar>(y,x)=255;
    }
  }
  // Trace a wall only where returns at two heights support it. The full
  // point map still checks every obstacle on the proposed flight segment.
  const auto kernel=cv::getStructuringElement(cv::MORPH_CROSS,cv::Size(3,3));
  cv::Mat near_below,near_above;
  cv::dilate(below,near_below,kernel);cv::dilate(above,near_above,kernel);
  cv::Mat hits=(below & near_above) | (above & near_below);
  if (!cv::countNonZero(hits)) return;
  cv::Mat distance;
  cv::distanceTransform(255-hits,distance,cv::DIST_L2,cv::DIST_MASK_PRECISE);
  cv::Mat occupied=distance<=.88f*safe/cell;
  vector<vector<cv::Point>> outlines;
  cv::findContours(occupied,outlines,cv::RETR_LIST,cv::CHAIN_APPROX_NONE);
  const auto position=[&](const cv::Point &q) {
    return Eigen::Vector3f((base.x()-half+q.x+.5f)*cell,(base.y()-half+q.y+.5f)*cell,z);
  };
  Eigen::Vector2f forward(std::cos(graph_->odom_node_->yaw_),std::sin(graph_->odom_node_->yaw_));
  if (const auto incoming=recentFlightDirection(body)) forward=*incoming;
  // A straight, observed right face permits a longer first observation.
  // Stop extending as soon as its side range changes or returns disappear.
  // This leaves an opening or a bend to the ordered contour sampler.
  const Eigen::Vector2f right(forward.y(),-forward.x());
  const auto sideRange=[&](const Eigen::Vector3f &point) {
    for (float r=.6f*safe;r<=3.f*safe;r+=cell) {
      for (float shift:{0.f,-cell,cell}) {
        const Eigen::Vector2f xy=point.head<2>()+r*right+shift*forward;
        const Eigen::Vector2i pixel=(xy/cell).array().floor().cast<int>().matrix()-base+Eigen::Vector2i(half,half);
        if (pixel.x()>=0 && pixel.x()<hits.cols && pixel.y()>=0 && pixel.y()<hits.rows &&
            hits.at<uchar>(pixel.y(),pixel.x())) return r;
      }
    }
    return 0.f;
  };
  // Compare observations over the same flight distance. A long run of
  // unflown air beside an already measured wall adds little information.
  // At a gap, a nearby face may expose new returns on both sides of the
  // guide. Use only the first measured surface on each ray.
  const auto freshFaces=[&](const Eigen::Vector3f &point,const Eigen::Vector2f &tangent,ClusterInfo &obs) {
    int fresh=0;
    for (float sign:{1.f,-1.f}) {
      const Eigen::Vector3f ray(sign*tangent.y(),-sign*tangent.x(),0.f);
      for (float r=cell;r<=frtp_.good_observation_trust_length_;r+=cell) {
        const Eigen::Vector3f wall=point+r*ray;
        const Eigen::Vector2i pixel=(wall.head<2>()/cell).array().floor().cast<int>().matrix()-base+Eigen::Vector2i(half,half);
        if (pixel.x()<0 || pixel.x()>=hits.cols || pixel.y()<0 || pixel.y()>=hits.rows) break;
        if (!hits.at<uchar>(pixel.y(),pixel.x())) continue;
        if (!observedFace(wall,-ray)) {
          ++fresh;
          obs.contour_support_.push_back({ObservationSample::Kind::WallFace,wall,-ray});
        }
        break;
      }
    }
    return fresh;
  };
  const auto faceYield=[&](const vector<Eigen::Vector3f> &edge,size_t first,size_t last,ClusterInfo &obs) {
    float length=0.f,gain=0.f;
    for (size_t i=first+1;i<=last;++i) {
      const Eigen::Vector3f delta=edge[i]-edge[i-1];
      const float step=delta.head<2>().norm();
      if (step<.05f) continue;
      // A join to an outline is transit. Only sample the measured sweep
      // itself, and give all observations the same local lookahead.
      const float use=std::min(step,5.f*safe-length);
      const Eigen::Vector2f tangent=delta.head<2>()/step;
      for (float a=0.f;a<use;a+=frtp_.cell_size_) {
        const float ds=std::min(frtp_.cell_size_,use-a);
        const Eigen::Vector3f point=edge[i-1]+((a+.5f*ds)/step)*delta;
        if (!visitedStance(point)) {
          const int faces=freshFaces(point,tangent,obs);
          // A small lateral offset can miss the flown voxel while staying
          // in the same visible strip. It earns coverage for new wall faces.
          const bool repeat=faces==0 && nearFlightPath(point,2.5f*graph_->bubble_min_radius_,
              frtp_.good_observation_trust_length_,cell);
          if (!repeat) gain+=ds*(.5f+float(faces));
        }
      }
      length+=use;
      if (length>=5.f*safe-.001f) break;
    }
    return gain;
  };
  TopoNode::Ptr straight_wall;
  const float start_range=sideRange(body);
  if (start_range>0.f) {
    vector<Eigen::Vector3f> straight{body};
    float unflown=0.f;
    for (float along=frtp_.cell_size_;along<frtp_.update_length_;along+=frtp_.cell_size_) {
      Eigen::Vector3f next=body;
      next.head<2>()+=along*forward;
      next.z()=straight.back().z()+std::clamp(z-straight.back().z(),-.25f*cell,.25f*cell);
      const float side=sideRange(next);
      if (side==0.f || std::abs(side-start_range)>std::max(2.f*cell,.45f*safe)) break;
      if (lidar_map_interface_->getDisToOcc(next)<minimum_clearance || !certifyFlightGuide({straight.back(),next})) break;
      straight.push_back(next);
      if (!visitedStance(next)) unflown+=frtp_.cell_size_;
    }
    if (unflown>4.f*safe && (straight.back()-body).norm()>6.f*safe &&
        !visitedStance(straight.back()) && certifyFlightGuide({body,straight.back()})) {
      auto node=std::make_shared<TopoNode>();node->is_viewpoint_=true;
      node->center_=straight.back();node->yaw_=std::atan2(forward.y(),forward.x());
      auto observation=std::make_shared<ClusterInfo>();
      observation->normal_=Eigen::Vector3f(-right.x(),-right.y(),0.f);
      observation->center_=node->center_-start_range*observation->normal_;
      observation->close_surface_sample_=!observedFace(observation->center_,observation->normal_);
      if (include_alternative) observation->fresh_contour_length_=faceYield(straight,0,straight.size()-1,*observation);
      // Lidar can measure a wall ahead before the body traverses it.
      // Keep that continuous, unflown edge until the wall turns or ends.
      const bool new_wall_pass=include_alternative &&
          std::none_of(straight.begin()+straight.size()/2,straight.end(),[&](const auto &point) {
            return nearFlightPath(point,1.5f*safe,0.f,cell);
          });
      if (!include_alternative || observation->fresh_contour_length_>=5.f*safe || new_wall_pass) {
        observation->contour_approach_={body,node->center_};
        observation->contour_preview_={node->center_};
        perimeter_target_=node->center_;
        observation->follows_local_outline_=true;
        observations_[node.get()]=observation;views.push_back(node);
        if (!include_alternative) return;
        straight_wall=node;
      }
    }
  }
  // Match only the local tangent of the last guide. The rest of that guide
  // can change as soon as a new right-hand opening becomes visible.
  Eigen::Vector3f anchor=body;
  float closest=2.f*safe,arc=0.f;
  for (size_t i=1;i<previous_outline_.size() && arc<7.f*safe;++i) {
    const auto &a=previous_outline_[i-1],&b=previous_outline_[i];
    const Eigen::Vector3f edge=b-a;const float length=edge.norm();
    if (length<.05f) continue;
    const float t=std::clamp((body-a).dot(edge)/edge.squaredNorm(),0.f,1.f);
    const Eigen::Vector3f q=a+t*edge;
    if ((q-body).norm()<closest && edge.head<2>().norm()>.1f) {
      closest=(q-body).norm();anchor=q;
      // The short vector from the body back to the wall is a lateral
      // correction. Estimate heading from the wall itself over a metre.
      size_t lead=i;
      while (lead+1<previous_outline_.size() &&
             (previous_outline_[lead]-a).norm()<safe) ++lead;
      const Eigen::Vector2f along=(previous_outline_[lead]-a).head<2>();
      forward=along.norm()>.1f ? along.normalized() : edge.head<2>().normalized();
    }
    arc+=length;
  }
  struct Start { size_t curve,index;float score; };
  vector<Start> starts,forward_starts;
  const auto tangentAt=[&](const vector<cv::Point> &curve,size_t i) -> Eigen::Vector2f {
    const auto delta=curve[(i+3)%curve.size()]-curve[(i+curve.size()-3)%curve.size()];
    return Eigen::Vector2f(delta.x,delta.y).normalized();
  };
  for (size_t k=0;k<outlines.size();++k) {
    const auto &curve=outlines[k];
    if (curve.size()<12) continue;
    Start best{k,0,1.e6f},ahead{k,0,1.e6f};
    for (size_t i=0;i<curve.size();i+=2) {
      const Eigen::Vector2f tangent=tangentAt(curve,i);
      if (!tangent.allFinite() || tangent.dot(forward)<-.55f) continue;
      Eigen::Vector3f q=position(curve[i]);
      q.head<2>()+=.5f*safe*Eigen::Vector2f(-tangent.y(),tangent.x());
      if ((q-body).norm()>4.f*safe) continue;
      if (!observedAir(q) && !observedAir(q+cell*Eigen::Vector3f::UnitZ()) &&
          !observedAir(q-cell*Eigen::Vector3f::UnitZ())) continue;
      const float score=(q-body).norm()+.8f*(q-anchor).norm()+2.f*safe*(1.f-tangent.dot(forward));
      if (score<best.score) best={k,i,score};
      const Eigen::Vector2f step=(q-body).head<2>();
      const float along=step.dot(forward);
      if (along>1.2f*safe && (step-along*forward).norm()<1.6f*safe &&
          tangent.dot(forward)>.2f && score<ahead.score) ahead={k,i,score};
    }
    if (best.score<1.e6f) starts.push_back(best);
    if (ahead.score<1.e6f && ahead.index!=best.index) forward_starts.push_back(ahead);
  }
  const auto lowerScore=[](const auto &a,const auto &b){return a.score<b.score;};
  std::sort(starts.begin(),starts.end(),lowerScore);
  std::sort(forward_starts.begin(),forward_starts.end(),lowerScore);
  std::unordered_set<size_t> blocked_starts;
  TopoNode::Ptr preferred;
  ClusterInfo::Ptr preferred_observation;
  float preferred_score=-std::numeric_limits<float>::infinity();
  std::optional<size_t> attached_curve;
  // Try the nearest wall normally, then a checked reattachment. Only when
  // that local outline ends can a forward seed bridge a break in the scan.
  for (int pass=0;pass<3;++pass) for (const auto &start:pass==2 ? forward_starts : starts) {
    const bool reattach=pass!=0;
    if (reattach && blocked_starts.empty()) break;
    const auto &seeds=pass==2 ? forward_starts : starts;
    // A separate, distant outline can be the opposite side of the passage.
    if (start.score>seeds.front().score+(pass==0 ? 4.f : 1.f)*safe) break;
    if (pass==1 && !blocked_starts.count(start.curve)) continue;
    const auto &curve=outlines[start.curve];
    vector<Eigen::Vector3f> route{body};
    size_t end=0;
    vector<bool> inspection{false};
    float length=0.f,unseen_arc=0.f;
    for (size_t j=0;j<curve.size() && length<frtp_.update_length_;j+=2) {
      const size_t index=(start.index+j)%curve.size();
      const auto &pixel=curve[index];
      if (pixel.x<2 || pixel.y<2 || pixel.x+2>=hits.cols || pixel.y+2>=hits.rows) break;
      const Eigen::Vector2f tangent=tangentAt(curve,index);
      if (!tangent.allFinite()) continue;
      const Eigen::Vector3f normal(-tangent.y(),tangent.x(),0.f);
      const Eigen::Vector3f edge=position(pixel);
      bool found=false;
      Eigen::Vector3f next;
      // The horizontal outline defines the wall side. Keep the flight height
      // when clear and test neighbouring observed heights at a low obstruction.
      vector<Eigen::Vector3f> join;
      const float previous_z=route.back().z();
      const float preferred_z=previous_z+std::clamp(z-previous_z,-cell,cell);
      for (float height:{preferred_z,previous_z,previous_z+cell,previous_z-cell,previous_z+2.f*cell,previous_z-2.f*cell}) {
        if (height<graph_->parallel_bubble_astar_->flight_min_z_ ||
            height>graph_->parallel_bubble_astar_->flight_max_z_) continue;
        for (float offset:{.5f,.35f,.65f,.2f,.8f,1.f,1.3f,1.6f}) {
          if (offset>1.f && !reattach) break;
          next=edge+offset*safe*normal;
          next.z()=height;
          if (lidar_map_interface_->getDisToOcc(next)<=minimum_clearance) continue;
          if ((next-route.back()).norm()<.15f) {next=route.back();found=true;break;}
          if (certifyFlightGuide({route.back(),next})) {found=true;break;}
        }
        if (found) break;
      }
      // At a protruding corner, the first directly visible stance may end
      // beside the obstruction. Rejoin a later stance on this same wall
      // through previously measured air, retaining the full checked join.
      if (!found && reattach && j*cell<5.f*safe && length<safe && (edge-body).norm()<4.f*safe) {
        next=edge+safe*normal;
        next.z()=z;
        if ((next-body).dot(Eigen::Vector3f(forward.x(),forward.y(),0.f))>-.5f*safe &&
            observedAir(next) && lidar_map_interface_->getDisToOcc(next)>minimum_clearance &&
            connectObserved(route.back(),next,.004,join) && join.size()>1 && certifyFlightGuide(join)) {
          for (size_t k=1;k+1<join.size();++k) {
            length+=(join[k]-route.back()).norm();
            route.push_back(join[k]);
            inspection.push_back(false);
          }
          found=true;
        }
      }
      if (!found) {
        // Sparse/rasterised returns can produce an isolated bad stance.
        // Skip a bounded part of the arc; the join itself must stay clear.
        if ((edge-route.back()).norm()>(reattach && length<safe ? 4.f:2.f)*safe) break;
        continue;
      }
      const float step=(next-route.back()).norm();
      if (step<.1f) continue;
      length+=step;route.push_back(next);
      const Eigen::Vector3f wall=edge-.88f*safe*normal;
      const bool seen=observedFace(wall,normal);
      unseen_arc=seen ? 0.f : unseen_arc+std::min(step,2.f*cell);
      // In a narrow return, the same body strip can put the other wall
      // on the right. Count that directed pass once using actual flight.
      const bool opposite=visitedStance(next) && needsOppositeTransit(next,tangent);
      const bool unflown=opposite || !visitedStance(next);
      const bool unseen=unflown && unseen_arc>=2.f*frtp_.cell_size_;
      inspection.push_back(unseen);
      if (length<=1.1f*safe) continue;
      if (!unflown) continue;
      // Follow arc order around the right wall. After a right turn even a
      // forward stance can lie behind the original heading in Cartesian space.
      if (opposite || unseen || !nearFlightPath(next,1.5f*safe)) {
        end=route.size()-1;
      }
    }
    if (std::getenv("EPIC_TRACE_WALL"))
      ROS_INFO("Right wall body %.2f %.2f forward %.2f %.2f score %.2f arc %.2f end %zu last %.2f %.2f z %.2f free %d clearance %.2f",
               body.x(),body.y(),forward.x(),forward.y(),start.score,length,end,route.back().x(),route.back().y(),
               body.z(),observedAir(body),lidar_map_interface_->getDisToOcc(body));
    if (!end) {
      if (!reattach && length<3.f*safe) blocked_starts.insert(start.curve);
      continue;
    }
    // Follow directly visible sections of the ordered wall. At a close bend
    // the last visible point is a transit point if the outgoing section has
    // also been measured. Give SUPER both sides of the turn in one guide.
    const auto visibleSection=[&](size_t first,float max_arc,float max_deviation) {
      size_t last=first;
      float arc=0.f;
      for (size_t i=first+1;i<=end;++i) {
        arc+=(route[i]-route[i-1]).norm();
        if (arc>max_arc) break;
        const Eigen::Vector3f chord=route[i]-route[first];
        if (chord.norm()<.3f*safe) continue;
        bool direct=certifyFlightGuide({route[first],route[i]});
        for (size_t j=first+1;direct && j<i;++j) {
          const auto step=route[j]-route[first];
          const float u=std::clamp(step.dot(chord)/chord.squaredNorm(),0.f,1.f);
          direct=(step-u*chord).norm()<=max_deviation;
        }
        if (direct) last=i;
      }
      return last;
    };
    size_t aim=visibleSection(0,std::numeric_limits<float>::infinity(),1.5f*safe);
    if (!aim) continue;
    const Eigen::Vector3f approach=route[aim]-body;
    if (aim<end && approach.norm()<5.f*safe) {
      const size_t next=visibleSection(aim,2.8f*safe,.5f*safe);
      const Eigen::Vector3f onward=route[next]-route[aim];
      if (onward.norm()>=.6f*safe && onward.dot(approach)>=-.2f*onward.norm()*approach.norm()) {
        if (std::getenv("EPIC_TRACE_WALL"))
          ROS_INFO("Measured turn at %.2f %.2f, outgoing guide %.2fm to %.2f %.2f",
                   route[aim].x(),route[aim].y(),onward.norm(),route[next].x(),route[next].y());
        aim=next;
      }
    }
    // Small closed outlines can lead away and return to the body in a
    // single local horizon. They cannot be a productive first observation.
    if ((route[aim]-body).norm()<graph_->bubble_min_radius_) continue;
    const bool inspect=inspection[aim];

    // Preserve the observed outgoing arc while keeping the actual observation as #1.
    size_t preview_end=aim;
    float preview_length=0.f;
    while (preview_end<end && preview_length<4.f*safe) {
      if ((route[preview_end+1]-body).norm()<graph_->bubble_min_radius_) break;
      preview_length+=(route[preview_end+1]-route[preview_end]).norm();
      ++preview_end;
    }
    auto node=std::make_shared<TopoNode>();
    node->center_=route[aim];node->is_viewpoint_=true;
    const Eigen::Vector3f edge=route[aim]-route[aim-1];
    node->yaw_=std::atan2(edge.y(),edge.x());
    auto obs=std::make_shared<ClusterInfo>();obs->close_surface_sample_=inspect;
    obs->normal_=Eigen::Vector3f(-std::sin(node->yaw_),std::cos(node->yaw_),0);
    obs->center_=node->center_-1.4f*safe*obs->normal_;
    obs->contour_preview_.assign(route.begin()+aim,route.begin()+preview_end+1);
    float unflown_length=0.f;
    // The lateral join to a different outline is transit and earns no coverage.
    for (size_t k=2;k<=preview_end;++k) {
      const float step=std::min((route[k]-route[k-1]).norm(),2.f*cell);
      const Eigen::Vector2f tangent=(route[k]-route[k-1]).head<2>();
      if (tangent.norm()<.05f) continue;
      if (!visitedStance(route[k]) || needsOppositeTransit(route[k],tangent.normalized()))
        unflown_length+=step;
      if (!inspection[k]) continue;
      obs->fresh_contour_length_+=step;
      const Eigen::Vector2f normal(-tangent.y(),tangent.x());
      Eigen::Vector3f wall=route[k];
      wall.head<2>()-=1.4f*safe*normal.normalized();
      obs->visible_cells_.emplace_back(wall.x(),wall.y(),wall.z());
    }
    obs->contour_approach_.assign(route.begin(),route.begin()+aim+1);
    // A crossing may be part of the outbound and inbound sweep. Keep it
    // when it leads to fresh air along this measured outline, but do not
    // create a new goal at the end of an already traversed pocket.
    if (unflown_length<cell && !inspect && visitedStance(node->center_) &&
        nearFlightPath(node->center_,1.6f*safe,3.f*safe,.4f*safe)) {
      float fresh_ahead=0.f;
      for (size_t k=preview_end+1;k<=end && fresh_ahead<safe;++k) {
        if (!visitedStance(route[k]))
          fresh_ahead+=std::min((route[k]-route[k-1]).norm(),2.f*cell);
      }
      if (fresh_ahead<safe) continue;
    }
    // A nearby detached face can continue the sweep when the current wall
    // leads back through already flown air. Compare the two before publishing
    // a single wall sample, and charge a new outline for the checked join.
    const float coverage=std::min(unflown_length,6.f*safe);
    const float score=coverage+.35f*std::min(obs->fresh_contour_length_,6.f*safe)-start.score;
    if (include_alternative) obs->fresh_contour_length_=faceYield(route,1,preview_end,*obs);
    if (score>preferred_score) {
      obs->follows_local_outline_=pass==0 && coverage>=2.f*safe &&
          (position(curve[start.index])-body).head<2>().norm()<=2.5f*safe;
      if (obs->follows_local_outline_)
        obs->fresh_contour_length_=std::max(obs->fresh_contour_length_,coverage);
      preferred_score=score;
      attached_curve=start.curve;
      preferred=node;
      preferred_observation=obs;
    }
    if (coverage>=2.f*safe || pass>0) break;
  }
  if (straight_wall && (!preferred || !preferred_observation->follows_local_outline_ ||
      ((preferred->center_-straight_wall->center_).norm()<graph_->bubble_min_radius_ &&
       preferred_observation->contour_preview_.size()<2))) return;
  if (preferred) {
    perimeter_target_=preferred->center_;
    views.push_back(preferred);
    observations_[preferred.get()]=preferred_observation;
  }
  // Allow the global tour to compare one nearby detached face before
  // passing its entrance. The rolling local sampler keeps its single goal.
  if (straight_wall || !include_alternative || (preferred && previous_outline_.empty())) return;
  vector<Start> nearby;
  for (size_t k=0;k<outlines.size();++k) {
    const auto &curve=outlines[k];if (curve.size()<16 || attached_curve==k) continue;
    Start best{k,0,std::numeric_limits<float>::infinity()};
    for (size_t i=0;i<curve.size();i+=5) {
      const auto tangent=tangentAt(curve,i);
      if (!tangent.allFinite()) continue;
      auto stance=position(curve[i]);
      stance.head<2>()+=.5f*safe*Eigen::Vector2f(-tangent.y(),tangent.x());
      const float distance=(stance-body).norm();
      if (distance<2.f*safe || distance>.75f*frtp_.update_length_ || visitedStance(stance) || !observedAir(stance)) continue;
      if (preferred && (stance-preferred->center_).norm()<2.f*safe) continue;
      const float score=distance+.8f*safe*(1.f-tangent.dot(forward));
      if (score<best.score) best={k,i,score};
    }
    if (std::isfinite(best.score)) nearby.push_back(best);
  }
  std::sort(nearby.begin(),nearby.end(),lowerScore);
  TopoNode::Ptr alternative;
  ClusterInfo::Ptr alternative_observation;
  float best_rate=0.f;
  bool best_continuation=false;
  const auto nextParallelPass=[&](const Eigen::Vector3f &goal) {
    if (!preferred || flight_history_.size()<4) return false;
    const float trust=frtp_.good_observation_trust_length_;
    float recent=0.f;
    Eigen::Vector3f latest=body;
    for (size_t end=flight_history_.size();end>1 && recent<2.5f*trust;) {
      const auto &turn=flight_history_[--end];
      recent+=(latest-turn).norm();latest=turn;
      if (recent<2.f*safe || std::abs(turn.z()-body.z())>safe) continue;
      size_t first=end;float arc=0.f;
      while (first>0 && arc<2.3f*trust) {
        arc+=(flight_history_[first]-flight_history_[first-1]).norm();--first;
      }
      Eigen::Vector2f axis=(turn-flight_history_[first]).head<2>();
      if (arc<2.f*trust || axis.norm()<.9f*arc) continue;
      axis.normalize();
      const Eigen::Vector2f side(axis.y(),-axis.x()),crossing=(body-turn).head<2>();
      if (std::abs(crossing.dot(side))<safe || crossing.dot(axis)>.8f*crossing.norm()) continue;
      const Eigen::Vector2f next=(goal-body).head<2>();
      const float lateral=(goal-turn).head<2>().dot(side);
      if (std::abs(lateral)<2.f*safe || std::abs(lateral)>1.65f*trust ||
          next.dot(axis)>-.75f*trust || (preferred->center_-body).head<2>().dot(axis)<.5f*next.dot(axis)) continue;
      // At the end of a pass an already measured continuous separator can
      // hide the next viewpoint until the vehicle crosses its tip. Compare
      // that new observation with continuing sideways along the end wall.
      bool supported=true;
      for (float back:{.7f*trust,1.2f*trust}) {
        bool hit=false;
        for (float w=cell;w<std::abs(lateral);w+=cell) {
          Eigen::Vector3f q=turn;q.head<2>()+=-back*axis+std::copysign(w,lateral)*side;
          if (lidar_map_interface_->getDisToOcc(q)<.55f*safe) {hit=true;break;}
        }
        if (!hit) {supported=false;break;}
      }
      if (supported) return true;
    }
    return false;
  };
  for (size_t trial=0;trial<std::min<size_t>(4,nearby.size());++trial) {
    const auto seed=nearby[trial];const auto &curve=outlines[seed.curve];
    vector<Eigen::Vector3f> edge;
    float length=0.f;
    for (size_t j=0;j<curve.size() && length<5.f*safe;j+=2) {
      const size_t i=(seed.index+j)%curve.size();
      const auto tangent=tangentAt(curve,i);if (!tangent.allFinite()) break;
      auto stance=position(curve[i]);
      stance.head<2>()+=.5f*safe*Eigen::Vector2f(-tangent.y(),tangent.x());
      if (visitedStance(stance) || !observedAir(stance) || lidar_map_interface_->getDisToOcc(stance)<minimum_clearance) break;
      if (!edge.empty()) {
        if (!certifyFlightGuide({edge.back(),stance})) break;
        length+=(edge.back()-stance).norm();
      }
      edge.push_back(stance);
    }
    if (length<2.5f*safe || (edge.back()-edge.front()).norm()<2.f*safe) continue;
    vector<Eigen::Vector3f> join;
    if (!connectObserved(body,edge.front(),.008,join) || !certifyFlightGuide(join)) continue;
    auto obs=std::make_shared<ClusterInfo>();
    // Mark the first stance clearly inside this sweep. Reaching the join
    // alone can mean the aircraft is still travelling across its entrance.
    float entry_arc=0.f;
    size_t entry=0;
    while (entry+1<edge.size() && entry_arc<1.5f*safe) {
      entry_arc+=(edge[entry+1]-edge[entry]).norm();
      ++entry;
    }
    obs->contour_entry_=edge[entry];
    obs->contour_approach_=join;
    obs->contour_approach_.insert(obs->contour_approach_.end(),edge.begin()+1,edge.end());
    obs->contour_preview_={edge.back()};
    obs->fresh_contour_length_=faceYield(edge,0,edge.size()-1,*obs);
    float transit=0.f;
    for (size_t j=1;j<join.size();++j) transit+=(join[j]-join[j-1]).norm();
    bool closes=preferred && transit<7.f*safe && closesBoundarySweep(obs->contour_approach_);
    if (closes) {
      // Finish the current measured edge before transferring across the gap.
      // Joining directly from the present pose can cut across the perimeter.
      auto boundary=preferred_observation->contour_approach_;
      const auto &preview=preferred_observation->contour_preview_;
      if (preview.size()>1) boundary.insert(boundary.end(),preview.begin()+1,preview.end());
      size_t nearest=0;
      for (size_t j=1;j<boundary.size();++j)
        if ((boundary[j]-edge.front()).squaredNorm()<(boundary[nearest]-edge.front()).squaredNorm()) nearest=j;
      closes=transit<2.7f*safe;
      if (nearest && (boundary[nearest]-edge.front()).norm()<3.f*safe) {
        boundary.resize(nearest+1);
        boundary.insert(boundary.end(),edge.begin(),edge.end());
        if (certifyFlightGuide(boundary)) {
          obs->contour_approach_=std::move(boundary);
          closes=true;
        }
      }
    }
    if (!closes && preferred && transit<5.f*safe && obs->fresh_contour_length_>2.f*safe) {
      Eigen::Vector2f motion=Eigen::Vector2f::Zero();
      for (auto it=flight_history_.rbegin();it!=flight_history_.rend();++it) {
        motion=(body-*it).head<2>();
        if (motion.norm()>1.3f*graph_->bubble_min_radius_) break;
      }
      // A wall can turn ninety degrees before its viewpoint. Compare the
      // actual departure from the body, not the bearing to that endpoint.
      const auto departure=[&](const vector<Eigen::Vector3f> &path) -> Eigen::Vector2f {
        for (size_t i=1;i<path.size();++i) {
          const Eigen::Vector2f offset=(path[i]-body).head<2>();
          if (offset.norm()>=1.6f*safe) return offset;
        }
        return (path.back()-body).head<2>();
      };
      const auto old=departure(preferred_observation->contour_approach_);
      const auto fresh=departure(obs->contour_approach_);
      if (motion.norm()>safe && old.dot(motion)<-.25f*old.norm()*motion.norm() &&
          fresh.dot(motion)>.25f*fresh.norm()*motion.norm()) {
        // A fragmented boundary can point the attached contour straight
        // back out over the entry. Continue to the productive face ahead
        // when the outgoing join would repeat part of the recent approach.
        for (const auto &q:preferred_observation->contour_approach_) {
          if ((q-body).norm()>safe && nearFlightPath(q,1.5f*safe,0.f,.5f*safe)) {
            closes=true;break;
          }
        }
      }
    }
    const float rate=obs->fresh_contour_length_/(length+transit);
    const bool continues=closes || (transit<7.f*safe && obs->fresh_contour_length_>safe &&
        nextParallelPass(edge.back()));
    if (best_continuation!=continues ? !continues : rate<=best_rate) continue;
    auto node=std::make_shared<TopoNode>();node->center_=edge.back();node->is_viewpoint_=true;
    const auto tangent=edge.back()-edge[edge.size()-2];
    node->yaw_=std::atan2(tangent.y(),tangent.x());
    obs->normal_=Eigen::Vector3f(-std::sin(node->yaw_),std::cos(node->yaw_),0.f);
    obs->center_=node->center_-1.4f*safe*obs->normal_;
    obs->close_surface_sample_=!observedFace(obs->center_,obs->normal_);
    best_rate=rate;best_continuation=continues;
    alternative=node;
    alternative_observation=obs;
  }
  if (alternative) {
    // Keep both observations in the tour. When the attached outline ends
    // back in a surveyed strip, give a new measurable face the first turn.
    const float swath=2.5f*graph_->bubble_min_radius_;
    const auto overlaps=[&](const Eigen::Vector3f &p) {
      return nearFlightPath(p,swath,frtp_.good_observation_trust_length_,cell);
    };
    const bool surveyed_exit=preferred && !preferred_observation->contour_entry_ &&
        unseenSupport(*preferred_observation).empty() &&
        !preferred_observation->contour_preview_.empty() &&
        overlaps(preferred_observation->contour_preview_.back()) &&
        !unseenSupport(*alternative_observation).empty() && !overlaps(alternative->center_);
    if ((best_continuation || surveyed_exit) && preferred) {
      perimeter_target_=alternative->center_;
      views.insert(views.begin(),alternative);
    } else views.push_back(alternative);
    observations_[alternative.get()]=alternative_observation;
  }
}

// Test the side of the flown connector. A wall visible from its far end
// does not establish that the intervening open floor follows a boundary.
float FrontierManager::rightWallSearchRange() const {
  const float radius=graph_->bubble_min_radius_;
  double preferred;
  nh_.param("FrontierManager/preferred_wall_clearance",preferred,double(radius));
  return std::max(3.f*radius,1.5f*float(preferred));
}

std::optional<float> FrontierManager::rightWallDistance(const Eigen::Vector3f &point,
    const Eigen::Vector2f &heading,float max_range) const {
  const float cell=frtp_.cell_size_,radius=graph_->bubble_min_radius_;
  const Eigen::Vector3f right(heading.y(),-heading.x(),0.f);
  for (float distance=radius;distance<=max_range+.25f*cell;distance+=.5f*cell) {
    const Eigen::Vector3f probe=point+distance*right;
    PointVector hits;
    vector<float> distances;
    lidar_map_interface_->KNN(PointType(probe.x(),probe.y(),probe.z()),1,hits,distances);
    if (hits.empty() || distances.front()>.64f*cell*cell) continue;
    if (inspectionSurface(*lidar_map_interface_,hits.front(),cell,std::min(.5f,frtp_.target_max_abs_normal_z_)))
      return distance;
    return {};
  }
  return {};
}

bool FrontierManager::followsRightWall(const vector<Eigen::Vector3f> &guide) const {
  const float radius=graph_->bubble_min_radius_,cell=frtp_.cell_size_;
  const float max_range=rightWallSearchRange();
  const float horizon=frtp_.good_observation_trust_length_+2.f*radius;
  float along=0.f,next=cell,run=0.f;
  for (size_t i=1;i<guide.size() && next<=horizon;++i) {
    const Eigen::Vector3f edge=guide[i]-guide[i-1];
    const float length=edge.head<2>().norm();
    if (length<.01f) continue;
    const Eigen::Vector2f tangent=edge.head<2>()/length;
    for (;next<=along+length && next<=horizon;next+=cell) {
      const Eigen::Vector3f q=guide[i-1]+((next-along)/length)*edge;
      if (std::abs(q.z()-guide.front().z())>radius) return false;
      run=rightWallDistance(q,tangent,max_range) ? run+cell : 0.f;
      if (run>=1.5f*radius) return true;
    }
    along+=length;
  }
  return false;
}

bool FrontierManager::keepsRightWall(const vector<Eigen::Vector3f> &guide,float lookahead) const {
  if (guide.size()<2) return false;
  const float cell=frtp_.cell_size_,max_range=rightWallSearchRange();
  const float horizon=std::max(3.f*cell,lookahead);
  int samples=0,supported=0;
  float along=0.f,next=cell;
  for (size_t i=1;i<guide.size() && next<=horizon;++i) {
    const Eigen::Vector3f edge=guide[i]-guide[i-1];
    const float length=edge.head<2>().norm();
    if (length<.01f) continue;
    const Eigen::Vector2f tangent=edge.head<2>()/length;
    for (;next<=along+length && next<=horizon;next+=cell) {
      const Eigen::Vector3f point=guide[i-1]+((next-along)/length)*edge;
      ++samples;
      if (rightWallDistance(point,tangent,max_range)) ++supported;
    }
    along+=length;
  }
  // Require support along the route. A single close glimpse of a wall near
  // the start does not justify ranking a later crossing as wall following.
  return samples>=3 && supported*3>=samples*2;
}

vector<Eigen::Vector3f> FrontierManager::previewRightWall(const Eigen::Vector3f &start,
    const Eigen::Vector2f &heading) const {
  if (heading.squaredNorm()<.01f) return {};
  const float radius=graph_->bubble_min_radius_,cell=frtp_.cell_size_;
  double preference;
  nh_.param("FrontierManager/preferred_wall_clearance",preference,double(radius));
  const float preferred=std::max(radius,float(preference));
  const float range=rightWallSearchRange();
  const float step=std::max(radius,2.f*cell);
  const int steps=std::min(8,int(frtp_.good_observation_trust_length_/step));
  vector<Eigen::Vector3f> guide{start};
  Eigen::Vector2f tangent=heading.normalized();
  float fresh=0.f;
  for (int i=0;i<steps;++i) {
    const float yaw=std::atan2(tangent.y(),tangent.x());
    float best=std::numeric_limits<float>::infinity();
    Eigen::Vector3f next;
    Eigen::Vector2f next_tangent;
    for (const float turn:{0.f,-.26f,-.52f,.26f,.52f}) {
      const Eigen::Vector2f direction(std::cos(yaw+turn),std::sin(yaw+turn));
      if (direction.dot(heading)<-.05f*heading.norm()) continue;
      Eigen::Vector3f point=guide.back();
      point.head<2>()+=step*direction;
      if (!lidar_map_interface_->IsInBox(point) || !observedAir(point) ||
          (point-start).head<2>().norm()<.5f*step) continue;
      const auto wall=rightWallDistance(point,direction,range);
      if (!wall || !certifyCruiseGuide({guide.back(),point})) continue;
      const float score=std::abs(*wall-preferred)+step*std::abs(turn);
      if (score<best) {
        best=score;
        next=point;
        next_tangent=direction;
      }
    }
    if (!std::isfinite(best)) break;
    if (!visitedStance(next)) fresh+=step;
    guide.push_back(next);
    tangent=next_tangent;
  }
  if (fresh<2.f*step) return {};
  return guide;
}

// Compare every departure in the same frame: the direction actually flown
// into this junction. A connector heading back out through the entrance
// cannot become a right turn by measuring it from its own first edge.
FrontierManager::Departure FrontierManager::surveyDeparture(const Eigen::Vector3f &body,
    const Eigen::Vector2f &heading,const vector<Eigen::Vector3f> &guide,bool wall,bool opening) const {
  if (guide.size()<2 || heading.squaredNorm()<.01f) return Departure::Transfer;
  const float radius=graph_->bubble_min_radius_;
  const float horizon=frtp_.good_observation_trust_length_+radius;
  const Eigen::Vector2f forward=heading.normalized(),right(forward.y(),-forward.x());
  bool advances=false;
  float travelled=0.f;
  Eigen::Vector3f previous=body;
  for (const auto &point:guide) {
    const Eigen::Vector3f edge=point-previous;
    const float length=edge.norm();
    if (length<.001f) continue;
    const float look=std::min(length,horizon-travelled);
    const int steps=std::max(1,int(std::ceil(look/radius)));
    for (int k=1;k<=steps;++k) {
      const Eigen::Vector3f q=previous+edge*(look/length*float(k)/steps);
      const Eigen::Vector2f delta=(q-body).head<2>();
      if (std::abs(q.z()-body.z())>radius) return Departure::Transfer;
      const float along=delta.dot(forward),side=delta.dot(right);
      if (along < -radius && !advances) return Departure::Backwards;
      if (opening && side>1.5f*radius && side>.45f*delta.norm() && along>=-radius && !visitedStance(q))
        return Departure::RightOpening;
      if (along>radius) advances=true;
    }
    travelled+=look;
    if (travelled>=horizon-.001f) break;
    previous=point;
  }
  if (!advances) return Departure::Transfer;
  return wall ? Departure::AlongWall : Departure::Forward;
}

bool FrontierManager::opposesIssuedRightTurn(const Eigen::Vector3f &body,const Eigen::Vector3f &proposal) const {
  if (!issued_right_turn_) return false;
  const auto &[tip,outgoing]=*issued_right_turn_;
  const float radius=graph_->bubble_min_radius_;
  return (tip-body).norm()<=frtp_.good_observation_trust_length_ && std::abs(body.z()-tip.z())<=radius &&
         (proposal-tip).head<2>().dot(outgoing)<=-2.f*radius;
}

TopoNode::Ptr FrontierManager::continueIssuedRightTurn(const Eigen::Vector3f &body,const Eigen::Vector3f &proposal) {
  if (!opposesIssuedRightTurn(body,proposal)) return {};
  const auto &[tip,outgoing]=*issued_right_turn_;
  const float radius=graph_->bubble_min_radius_;
  // The next scan sees farther into the turn. Extend the issued entrance
  // along its original side before accepting a candidate on the opposite arm.
  vector<Eigen::Vector3f> onward,join;
  const auto band=currentSurveyBand(body);
  const bool found=observedBoundaryGuide(tip,onward,&outgoing,nullptr,2.f*radius,nullptr,band);
  surface_body_=body;
  if (!found || onward.size()<2 || visitedStance(onward.back())) return {};
  const Eigen::Vector2f normal(-outgoing.y(),outgoing.x());
  if (!std::all_of(onward.begin(),onward.end(),[&](const Eigen::Vector3f &p) {
        const Eigen::Vector2f delta=(p-tip).head<2>();
        return delta.dot(outgoing)>=-radius && std::abs(delta.dot(normal))<=2.f*radius &&
               std::abs(p.z()-tip.z())<=radius;
      })) return {};
  if (certifyFlightGuide({body,tip})) join={body,tip};
  else if (!connectObserved(body,tip,.01,join)) return {};
  join.insert(join.end(),onward.begin()+1,onward.end());
  if (!certifyFlightGuide(join)) return {};
  auto view=boundaryObservation(std::move(join));
  if (view) {
    auto &observation=*observations_.at(view.get());
    observation.contour_entry_=onward.back();
    observation.right_hand_opening_=true;
  }
  return view;
}

void FrontierManager::confirmWallGuide(const vector<Eigen::Vector3f> &approach,
                                      const vector<Eigen::Vector3f> &preview,bool has_entry) {
  const float radius=graph_->bubble_min_radius_;
  if (approach.size()<2 || (issued_right_turn_ && (issued_right_turn_->first-approach.back()).norm()>.1f))
    issued_right_turn_.reset();
  if (has_entry && !issued_right_turn_ && approach.size()>1) {
    const auto motion=recentFlightDirection(approach.front());
    if (motion) {
      const Eigen::Vector2f right(motion->y(),-motion->x());
      const Eigen::Vector3f offset=approach.back()-approach.front();
      const Eigen::Vector2f last=(approach.back()-approach[approach.size()-2]).head<2>();
      if (std::abs(offset.z())<=radius && offset.head<2>().dot(right)>1.5f*radius &&
          offset.head<2>().dot(*motion)>=-radius && last.dot(right)>.5f*last.norm())
        issued_right_turn_=std::make_pair(approach.back(),right);
    }
  }
  previous_outline_=approach;
  if (preview.size()>1 && !approach.empty() && (approach.back()-preview.front()).norm()<.1f)
    previous_outline_.insert(previous_outline_.end(),preview.begin()+1,preview.end());
}

size_t FrontierManager::heightBoundedPrefix(const vector<Eigen::Vector3f> &path,double margin) {
  if (path.size()<2 || margin<0.) return path.size();
  const float start=path.front().z();
  float low=start,high=start;
  size_t end=1;
  for (size_t i=1;i<path.size();++i) {
    const float z=path[i].z();
    low=std::min(low,z);high=std::max(high,z);
    if (low>=std::min(start,z)-margin && high<=std::max(start,z)+margin) end=i+1;
  }
  return end;
}

bool FrontierManager::centredForwardPass(const Eigen::Vector3f &body,const Eigen::Vector2f &along,
                                       vector<Eigen::Vector3f> &path) {
  const float cell=frtp_.cell_size_,radius=graph_->bubble_min_radius_,trust=frtp_.good_observation_trust_length_;
  auto flight=surveyFlightDirection(body);
  const bool short_history=!flight.has_value();
  if (!flight || flight->dot(along)<.85f) {
    float length=0.f;
    Eigen::Vector3f previous=body;
    for (auto it=flight_history_.rbegin();it!=flight_history_.rend();++it) {
      length+=(previous-*it).norm();previous=*it;
      if (length<2.5f*radius) continue;
      const Eigen::Vector2f chord=(body-*it).head<2>();
      if (chord.norm()>.85f*length && std::abs(it->z()-body.z())<radius) flight=chord.normalized();
      break;
    }
  }
  if (!flight || flight->dot(along)<.78f) return false;
  const double now=epic_ros::Time::now().toSec();
  if (failed_centred_pass_ && (body-failed_centred_pass_->first).norm()<radius &&
      now>=failed_centred_pass_->second && now-failed_centred_pass_->second<1.) return false;
  const Eigen::Vector2f left(-flight->y(),flight->x());
  // Measure the full span. An offset body need not be equally close to
  // the two faces that delimit the same forward sweep.
  const auto measured_span=[&](Eigen::Vector3f centre) {
    float width=0.f;
    for (float sign:{-1.f,1.f}) {
      float reach=.5f*cell;
      for (;reach<=trust;reach+=.5f*cell) {
        Eigen::Vector3f q=centre;
        q.head<2>()+=sign*reach*left;
        if (lidar_map_interface_->getDisToOcc(q)<cell) break;
      }
      if (reach>trust) return 0.f;
      width+=reach;
    }
    return width<=trust+2.f*radius ? width:0.f;
  };
  const float span=measured_span(body);
  if (span==0.f) return false;
  if (short_history) {
    Eigen::Vector3f next=body;
    next.head<2>()+=2.f*radius*(*flight);
    if (measured_span(next)==0.f) return false;
  }
  const float ahead=std::min(.6f*frtp_.update_length_,2.f*trust-radius);
  const float step=.25f*cell,lateral_range=std::min(.5f*span,trust*.5f);
  const float vertical_range=3.f*cell;
  vector<Eigen::Vector3f> ridge;
  double height_margin;
  nh_.param("bubble_astar/endpoint_height_margin",height_margin,-1.);
  const auto productive=[&](float fresh_required,float advance_required) {
    ridge.resize(heightBoundedPrefix(ridge,height_margin));
    if (ridge.size()<2) return false;
    float fresh_tail=0.f;
    for (size_t i=1;i<ridge.size();++i) {
      if (visitedStance(ridge[i])) fresh_tail=0.f;
      else fresh_tail+=(ridge[i]-ridge[i-1]).norm();
    }
    return fresh_tail>=fresh_required && (ridge.back()-body).head<2>().dot(along)>=advance_required;
  };
  const auto reach_slice=[&](float distance,float minimum) {
    Eigen::Vector3f centre=body;
    centre.head<2>()+=distance*(*flight);
    centre.z()=.5f*(body.z()+survey_height_.value_or(body.z()));
    Eigen::Vector3f goal=centre;
    float best=-std::numeric_limits<float>::infinity();
    for (float side=-lateral_range;side<=lateral_range+.1f*step;side+=step) {
      for (float rise=-vertical_range;rise<=vertical_range+.1f*step;rise+=step) {
        Eigen::Vector3f q=centre;
        q.head<2>()+=side*left;q.z()+=rise;
        if (!isInBox(q) || !observedAir(q) || visitedStance(q)) continue;
        const float clearance=lidar_map_interface_->getDisToOcc(q);
        if (clearance<measuredStanceClearance()) continue;
        const float score=std::min(clearance,radius+cell)-.025f*std::abs(side)-.1f*std::abs(rise);
        if (score>best) {best=score;goal=q;}
      }
    }
    return std::isfinite(best) && connectDistantObserved(body,goal,ridge) && certifyFlightGuide(ridge) &&
        productive(minimum,std::min(distance-cell,trust));
  };
  if (!reach_slice(ahead,trust) &&
      (!observedBoundaryGuide(body,ridge,&along,nullptr,trust+2.f*radius) || !productive(trust,trust))) {
    // At the limit of a measured sightline the distant stance can be
    // disconnected. Observe from its unflown near side before reversing.
    // The caller also requires distinct new surface or unknown-air samples.
    if (!reach_slice(trust,2.f*radius) && !reach_slice(.5f*trust,1.25f*radius)) {
      failed_centred_pass_=std::make_pair(body,now);
      return false;
    }
  }
  failed_centred_pass_.reset();
  path.swap(ridge);
  return true;
}

// Measure the information that a forward observation could add after a wall
// ends. The entire approach stays in lidar-confirmed air. A small sideways
// displacement allows a continuous pass around a protruding measured obstacle.
TopoNode::Ptr FrontierManager::forwardObservation(const Eigen::Vector3f &body,bool require_broad) {
  if (flight_history_.size()<3) return {};
  const float radius=graph_->bubble_min_radius_, cell=frtp_.cell_size_;
  Eigen::Vector2f along=Eigen::Vector2f::Zero();
  for (auto p=flight_history_.rbegin();p!=flight_history_.rend();++p) {
    along=(body-*p).head<2>();
    if (along.norm()>2.f*radius) break;
  }
  if (along.norm()<radius) return {};
  along.normalize();
  // Use a longer baseline while a straight pass is still under way. A
  // short lateral correction must not rotate the survey line on every scan.
  float run=0.f;
  Eigen::Vector3f last=body;
  for (auto it=flight_history_.rbegin();it!=flight_history_.rend();++it) {
    run+=(*it-last).norm();last=*it;
    if (run<2.f*frtp_.good_observation_trust_length_) continue;
    const Eigen::Vector2f chord=(body-last).head<2>();
    if (chord.norm()>.93f*run && chord.dot(along)>.9f*chord.norm()) along=chord.normalized();
    break;
  }
  const Eigen::Vector2f left(-along.y(),along.x());
  const float trust=frtp_.good_observation_trust_length_;
  vector<Eigen::Vector3f> best;
  float bypass_tail=0.f;
  float best_advance=0.f;
  vector<std::pair<float,float>> tested_offsets;
  for (const float offset:{0.f,-radius,radius,-1.75f*radius,1.75f*radius}) {
    vector<Eigen::Vector3f> path{body};
    float fresh=0.f,total=0.f;
    for (float s=cell;s<=frtp_.update_length_;s+=cell) {
      const float phase=std::min(1.f,s/(3.f*radius));
      Eigen::Vector3f next=body;
      next.head<2>()+=s*along+offset*(phase*phase*(3.f-2.f*phase))*left;
      if (survey_height_) next.z()+=std::clamp(*survey_height_-body.z(),-.12f*s,.12f*s);
      if (!isInBox(next) || !certifyFlightGuide({path.back(),next})) break;
      path.push_back(next);
      fresh+=visitedStance(next) ? 0.f : cell;
      total=s;
    }
    const float advance=fresh-std::abs(offset);
    tested_offsets.emplace_back(total,offset);
    if (advance>best_advance && fresh>.65f*total && total>=6.f*radius) {
      best=std::move(path);best_advance=advance;
    }
    if (offset==0.f && best_advance>frtp_.update_length_-2.f*cell) break;
  }
  if (best_advance<.6f*frtp_.update_length_) {
    // The forward sightline can end at a local obstacle even though the
    // previously scanned volume continues on its other side.
    const auto started=std::chrono::steady_clock::now();
    std::stable_sort(tested_offsets.begin(),tested_offsets.end(),[](const auto &a,const auto &b) {
      return a.first>b.first;
    });
    for (float ahead:{frtp_.update_length_-cell,.8f*frtp_.update_length_}) {
      if (best_advance>=.6f*frtp_.update_length_) break;
      for (const auto &tested:tested_offsets) {
        const float offset=tested.second;
        Eigen::Vector3f goal=body;
        goal.head<2>()+=ahead*along+offset*left;
        if (survey_height_) goal.z()=*survey_height_;
        if (!isInBox(goal) || !observedAir(goal) || visitedStance(goal)) continue;
        const double budget=.065-std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
        if (budget<=0.) break;
        vector<Eigen::Vector3f> path;
        if (!connectObserved(body,goal,std::min(.032,budget),path) || !certifyFlightGuide(path)) continue;
        vector<Eigen::Vector3f> sampled{body};
        float length=0.f,fresh=0.f,fresh_tail=0.f;
        for (size_t i=1;i<path.size();++i) {
          const Eigen::Vector3f edge=path[i]-path[i-1];
          const float span=edge.norm();
          const int steps=std::max(1,int(std::ceil(span/cell)));
          for (int j=1;j<=steps;++j) {
            sampled.push_back(path[i-1]+float(j)/steps*edge);
            if (visitedStance(sampled.back())) fresh_tail=0.f;
            else {fresh+=span/steps;fresh_tail+=span/steps;}
          }
          length+=span;
        }
        const float advance=fresh-(length-(body-goal).norm())-std::abs(offset);
        // A short known approach may be the only way around a measured
        // obstruction. Count the uninterrupted new stretch on its far side.
        if (length<1.35f*frtp_.update_length_ && (fresh>.5f*length || fresh_tail>=trust) &&
            advance>best_advance) {
          best=std::move(sampled);best_advance=advance;
          bypass_tail=fresh_tail;
          break;
        }
      }
    }
  }
  bool centred=false;
  if (!require_broad && best_advance<3.f*radius) {
    vector<Eigen::Vector3f> pass;
    if (centredForwardPass(body,along,pass)) {
      best=std::move(pass);
      centred=true;
    }
  }
  return evaluateSurveyPath(body,along,std::move(best),bypass_tail,centred,require_broad);
}

// Sample the unflown end of a measured strip even between updates of the
// frontier clusters. All guide segments remain in the certified component.
TopoNode::Ptr FrontierManager::rollingSurveyView(const Eigen::Vector3f &body,const Eigen::Vector2f &direction) {
  vector<Eigen::Vector3f> route;
  if (centredForwardPass(body,direction,route) && route.size()>1 && !visitedStance(route.back())) {
    if (auto pass=evaluateSurveyPath(body,direction,std::move(route),0.f,true,false)) return pass;
  }
  // A wall can hide the end of a strip while the lidar has already exposed
  // a connected free/unknown boundary beyond it. Sample its certified stance.
  if (observedBoundaryGuide(body,route,&direction,nullptr,3.f*graph_->bubble_min_radius_))
    return boundaryObservation(std::move(route));
  // A narrow section can hide every ordinary endpoint beyond it. Sample one
  // measured stance on the continuing wall before selecting a return route.
  const float radius=graph_->bubble_min_radius_,cell=frtp_.cell_size_;
  const float trust=frtp_.good_observation_trust_length_;
  if (aperture_clearance_<=0. || !survey_height_ || std::abs(*survey_height_-body.z())>radius ||
      !hasLateralSurveyGain(body,true)) return {};
  const Eigen::Vector2f side(direction.y(),-direction.x());
  for (float reach : {2.f*trust,trust}) {
    Eigen::Vector3f best=body;
    float margin=measuredStanceClearance();
    for (int h=-2;h<=2;++h) for (int k=-3;k<=3;++k) {
      Eigen::Vector3f point=body;
      point.head<2>()+=reach*direction+.5f*cell*float(k)*side;
      point.z()+=cell*float(h);
      if (!withinFlightHeight(point) || !observedAir(point) || std::abs(point.z()-body.z())>radius) continue;
      const float space=lidar_map_interface_->getDisToOcc(point);
      if (space>margin) {margin=space;best=point;}
    }
    if (margin<=measuredStanceClearance() || visitedStance(best) || flownObservationStance(best) ||
        !hasLateralSurveyGain(best,true) || !followsRightWall({body,best}) ||
        !connectObservedSections(body,best,route,true)) continue;
    auto view=std::make_shared<TopoNode>();
    view->is_viewpoint_=true;view->center_=best;view->yaw_=std::atan2(direction.y(),direction.x());
    auto observation=std::make_shared<ClusterInfo>();
    observation->center_=best;observation->normal_=Eigen::Vector3f(-side.x(),-side.y(),0.f);
    observation->follows_local_outline_=true;observation->aperture_approach_=true;
    observation->fresh_contour_length_=std::min(reach,trust);
    observation->contour_preview_={best};observation->contour_approach_=route;
    observations_[view.get()]=observation;
    last_aperture_path_=std::move(route);
    ROS_INFO("Sampled an unflown wall across a checked aperture, %.2fm ahead",reach);
    return view;
  }
  return {};
}

// A cluster centroid can miss the sparse topology while an individual lidar
// viewpoint has a connected approach. Keep one productive pass in the tour.
TopoNode::Ptr FrontierManager::sampledSurveyPass(const Eigen::Vector3f &body,const PointVector &samples) {
  const float radius=graph_->bubble_min_radius_,cell=frtp_.cell_size_;
  if (samples.empty() || !survey_height_) return {};
  const float level=body.z()+std::clamp(*survey_height_-body.z(),-cell,cell);
  const auto heading=recentFlightDirection(body);
  if (!heading) return {};
  std::vector<std::pair<float,Eigen::Vector3f>> ranked;
  std::unordered_set<Eigen::Vector3i,Vector3i_Hash> unique;
  for (const auto &sample:samples) {
    const Eigen::Vector3f goal=sample.getVector3fMap();
    const Eigen::Vector2f offset=(goal-body).head<2>();
    const float distance=offset.norm();
    if (distance<3.f*radius || distance>frtp_.update_length_ ||
        std::abs(goal.z()-level)>radius || offset.dot(*heading)<.6f*distance ||
        visitedStance(goal) || !observedAir(goal) || lidar_map_interface_->getDisToOcc(goal)<measuredStanceClearance()) continue;
    if (unique.insert((goal/cell).array().floor().cast<int>()).second)
      ranked.emplace_back(offset.dot(*heading),goal);
  }
  std::stable_sort(ranked.begin(),ranked.end(),[](const auto &a,const auto &b){return a.first>b.first;});
  // Prefer inexpensive connections. Spend one fine search on the leading
  // viewpoint when the sparse grid missed a narrow part of its approach.
  for (size_t i=0;i<std::min<size_t>(ranked.size(),4);++i) {
    const auto &goal=ranked[i].second;
    vector<Eigen::Vector3f> route;
    if (!connectObserved(body,goal,.016,route) && (i!=0 || !connectDistantObserved(body,goal,route))) continue;
    if (route.size()<2 || !certifyFlightGuide(route) ||
        std::any_of(route.begin(),route.end(),[&](const auto &p){return std::abs(p.z()-level)>radius;})) continue;
    auto pass=evaluateSurveyPath(body,(goal-body).head<2>().normalized(),std::move(route),0.f,true,false);
    if (pass) {
      ROS_INFO("Recovered sampled survey viewpoint with %zu fresh observations",observations_.at(pass.get())->contour_support_.size());
      return pass;
    }
  }
  return {};
}

// A distant frontier can be visible from above an unfinished pass. Test the
// measured approach at survey height for observations of its own before
// climbing. The elevated viewpoint remains a separate candidate in the tour.
TopoNode::Ptr FrontierManager::surveyApproach(const Eigen::Vector3f &body,const Eigen::Vector3f &target) {
  const float radius=graph_->bubble_min_radius_,cell=frtp_.cell_size_;
  if (!survey_height_ || std::abs(body.z()-*survey_height_)>radius ||
      std::abs(target.z()-*survey_height_)<=radius) return {};
  const auto heading=surveyFlightDirection(body);
  const Eigen::Vector2f offset=(target-body).head<2>();
  const float horizontal=offset.norm();
  const float trust=frtp_.good_observation_trust_length_;
  if (!heading || horizontal<trust || horizontal>1.5f*frtp_.update_length_) return {};
  const Eigen::Vector2f direction=offset/horizontal;
  if (direction.dot(*heading)<.9f) {
    // During a bend the long chord lags the actual flight direction. A
    // recent measured chord can confirm a turn toward this same frontier.
    if (direction.dot(*heading)<.7f) return {};
    Eigen::Vector2f recent=Eigen::Vector2f::Zero();
    for (auto it=flight_history_.rbegin();it!=flight_history_.rend();++it) {
      recent=(body-*it).head<2>();
      if (recent.norm()>=2.f*radius) break;
    }
    if (recent.norm()<2.f*radius || direction.dot(recent)<.9f*recent.norm()) return {};
  }
  Eigen::Vector3f goal=target;goal.z()=*survey_height_;
  if (!observedAir(goal) || visitedStance(goal)) return {};
  vector<Eigen::Vector3f> route;
  if ((!connectObserved(body,goal,.025,route) && !connectDistantObserved(body,goal,route)) ||
      !certifyFlightGuide(route)) return {};
  vector<Eigen::Vector3f> sampled{body};
  float fresh_tail=0.f,unflown=0.f,total=0.f;
  for (size_t i=1;i<route.size();++i) {
    if (std::abs(route[i].z()-*survey_height_)>radius) return {};
    const Eigen::Vector3f edge=route[i]-route[i-1];
    const float length=edge.norm();
    total+=length;
    if (total>1.4f*horizontal) return {};
    const int steps=std::max(1,int(std::ceil(length/cell)));
    for (int j=1;j<=steps;++j) {
      sampled.push_back(route[i-1]+edge*(float(j)/steps));
      if (visitedStance(sampled.back())) fresh_tail=0.f;
      else {fresh_tail+=length/steps;unflown+=length/steps;}
    }
  }
  // Crossing an old track partway through this leg can split a productive
  // swath. Retain a fresh exit and require measurable gain along the path.
  if (fresh_tail<trust && (fresh_tail<2.f*radius || unflown<trust || unflown<.5f*total)) return {};
  return evaluateSurveyPath(body,direction,std::move(sampled),fresh_tail,false,false);
}

TopoNode::Ptr FrontierManager::evaluateSurveyPath(const Eigen::Vector3f &body,const Eigen::Vector2f &along,
    vector<Eigen::Vector3f> best,float bypass_tail,bool centred,bool require_broad) {
  const float cell=frtp_.cell_size_,radius=graph_->bubble_min_radius_,trust=frtp_.good_observation_trust_length_;
  const Eigen::Vector2f left(-along.y(),along.x());
  if (best.empty()) return {};
  float pass_length=trust;
  if (centred) {
    // Certified routes can have long edges. Evaluate visibility at metric
    // intervals independently of how many vertices the connector produced.
    vector<Eigen::Vector3f> sampled{best.front()};
    pass_length=0.f;
    for (size_t i=1;i<best.size();++i) {
      const Eigen::Vector3f edge=best[i]-best[i-1];
      const float length=edge.norm();
      const int steps=std::max(1,int(std::ceil(3.f*length/cell)));
      for (int j=1;j<=steps;++j) {
        sampled.push_back(best[i-1]+edge*(float(j)/steps));
        if (!visitedStance(sampled.back())) pass_length+=length/steps;
      }
    }
    best.swap(sampled);
  }
  std::unordered_set<Eigen::Vector3i,Vector3i_Hash> unknown,weak_surfaces,tested_hits;
  const bool wall_targets=frtp_.target_max_abs_normal_z_<1.f;
  std::vector<ObservationSample> samples;
  float footprint=0.f,broad_run=0.f,longest_broad=0.f;
  for (size_t j=2;j<best.size();j+=3) {
    if (visitedStance(best[j])) {broad_run=0.f;continue;}
    float width=0.f,smaller_side=trust;
    for (float side:{-1.f,1.f}) {
      float reach=0.f;
      for (float w=cell;w<=trust;w+=cell) {
        Eigen::Vector3f q=best[j];q.head<2>()+=side*w*left;
        if (!observedAir(q) || lidar_map_interface_->getDisToOcc(q)<radius) break;
        reach=w;
      }
      width+=reach;smaller_side=std::min(smaller_side,reach);
    }
    const float ds=(best[j]-best[j>=3 ? j-3 : 0]).norm();
    footprint+=width*ds;
    broad_run=smaller_side>=.65f*trust && width>=1.6f*trust ? broad_run+ds : 0.f;
    longest_broad=std::max(longest_broad,broad_run);
    for (float angle:{-.95f,-.55f,0.f,.55f,.95f}) for (float slope:{-.9f,-.18f,0.f,.18f}) {
      Eigen::Vector3f ray;ray.head<2>()=std::cos(angle)*along+std::sin(angle)*left;
      ray.z()=slope;ray.normalize();
      for (float length=cell;length<1.5f*trust;length+=cell) {
        const Eigen::Vector3f q=best[j]+length*ray;
        if (!isInBox(q)) break;
        PointVector near;vector<float> distance;
        lidar_map_interface_->KNN(PointType(q.x(),q.y(),q.z()),1,near,distance);
        if (!near.empty() && distance.front()<cell*cell) {
          // A long-range return can mark the air as known while the
          // surface itself still lacks a close, dense observation.
          const Eigen::Vector3f hit=near.front().getVector3fMap();
          const Eigen::Vector3i patch=(hit/(2.f*cell)).array().floor().cast<int>();
          const auto it=inspected_faces_.find(patch);
          const bool any_face=!wall_targets && slope<-.5f;
          const bool inspected=any_face ? (it!=inspected_faces_.end() && it->second) : observedFace(hit,-ray);
          const Eigen::Vector3i fine=(hit/(.5f*cell)).array().floor().cast<int>();
          if (!inspected && !weak_surfaces.count(patch) && (!wall_targets || tested_hits.insert(fine).second) &&
              inspectionSurface(*lidar_map_interface_,near.front(),cell,frtp_.target_max_abs_normal_z_)) {
            weak_surfaces.insert(patch);
            samples.push_back({any_face ? ObservationSample::Kind::AnyFace : ObservationSample::Kind::WallFace,hit,-ray});
          }
          break;
        }
        const Eigen::Vector3f beyond=q+cell*ray;
        if (!observedAir(q) && !observedAir(beyond)) {
          if ((!wall_targets || std::abs(beyond.z()-best[j].z())<=radius) && isInBox(beyond) &&
              lidar_map_interface_->getDisToOcc(beyond)>cell &&
              unknown.insert((q/(2.f*cell)).array().floor().cast<int>()).second)
            samples.push_back({ObservationSample::Kind::UnknownAir,q});
          break;
        }
      }
    }
  }
  // Only choose this observation if it can reveal several distinct unknown
  // cells and its unflown footprint spans at least a near-range lidar swath.
  const float patch_area=4.f*cell*cell;
  const bool new_volume=unknown.size()>=3 && footprint>=2.f*trust*trust;
  const bool new_surface=weak_surfaces.size()*patch_area>=.5f*trust*trust && footprint>=trust*trust;
  // A narrow, measured passage behind an obstruction has a smaller visible
  // footprint than open floor. Require a full new length on its far side,
  // enough observed width for flight, and distinct uninspected patches.
  const float strip_area=2.f*radius*trust;
  const bool new_strip=bypass_tail>=trust && footprint>=strip_area &&
      (unknown.size()>=3 || weak_surfaces.size()*patch_area>=std::max(3.f*patch_area,.5f*strip_area)) &&
      !certifyFlightGuide({body,best.back()});
  const float pass_area=std::max(3.f*patch_area,radius*std::min(pass_length,trust));
  const bool new_pass=centred && (unknown.size()>=3 || weak_surfaces.size()*patch_area>=pass_area);
  if ((!new_volume && !new_surface && !new_strip && !new_pass) || (require_broad && longest_broad<trust)) return {};
  auto node=std::make_shared<TopoNode>();node->is_viewpoint_=true;node->center_=best.back();
  node->yaw_=std::atan2(along.y(),along.x());
  auto observation=std::make_shared<ClusterInfo>();
  observation->center_=best.back();
  observation->normal_=Eigen::Vector3f(-along.x(),-along.y(),0.f);
  observation->contour_approach_=std::move(best);
  observation->contour_preview_={node->center_};
  observation->fresh_contour_length_=centred ? std::min(trust,pass_length) :
      std::min(5.f*radius,footprint/(2.f*trust));
  observation->contour_support_=std::move(samples);
  observations_[node.get()]=observation;
  return node;
}

// A long, level observed pass can end against a surface while there is
// still a broad, unflown footprint alongside it. One rolling observation
// covers the turn and the start of that neighbouring footprint.
TopoNode::Ptr FrontierManager::parallelObservation(const Eigen::Vector3f &body,bool require_separator) {
  const float cell=frtp_.cell_size_, trust=frtp_.good_observation_trust_length_;
  const float radius=graph_->bubble_min_radius_;
  if (!survey_height_ || std::abs(*survey_height_-body.z())>radius || trust<=2.f*radius) return {};
  Eigen::Vector3f past=body;
  Eigen::Vector2f recent=Eigen::Vector2f::Zero();
  float travelled=0.f;
  for (auto it=flight_history_.rbegin();it!=flight_history_.rend() && travelled<2.5f*trust;++it) {
    if (std::abs(body.z()-it->z())>1.5f*radius) break;
    travelled+=(*it-past).norm();past=*it;
    if (recent.norm()<2.f*radius) recent=(body-past).head<2>();
  }
  Eigen::Vector2f along=(body-past).head<2>();
  if (travelled<2.f*trust || along.norm()<.85f*travelled) return {};
  along.normalize();
  // A long straight ending in a newly acquired corner must not launch a
  // synthetic U-turn across the corner while the body is already turning.
  if (recent.dot(along)<.9f*recent.norm()) return {};
  const Eigen::Vector2f right(along.y(),-along.x());
  const auto stance=[&](float longitudinal,float lateral) {
    Eigen::Vector3f q=body;
    q.head<2>()+=longitudinal*along+lateral*right;
    q.z()+=std::clamp(*survey_height_-body.z(),-.12f*std::hypot(longitudinal,lateral),
        .12f*std::hypot(longitudinal,lateral));
    return q;
  };
  const auto barrierRange=[&](float lateral) {
    for (float s=cell;s<=2.5f*trust;s+=cell) {
      const auto q=stance(s,lateral);
      if (lidar_map_interface_->getDisToOcc(q)<radius) return s;

    }
    return 0.f;
  };
  const float ahead=barrierRange(0.f);
  if (ahead<=0.f) return {};
  for (float sign:{1.f,-1.f}) for (float width:{1.1f*trust,trust,1.3f*trust,.8f*trust}) {
    const float middle=barrierRange(sign*.5f*width),far=barrierRange(sign*width);
    if (far<=0.f) continue;
    bool wide=true,separated=true;
    for (float s:{-trust,-.5f*trust}) {
      const auto a=stance(s,0.f),b=stance(s,sign*width);
      wide=wide && certifyFlightGuide({a,stance(s,sign*1.25f*width)});
      bool hit=false;
      for (float w=cell;w<width;w+=cell) {
        if (lidar_map_interface_->getDisToOcc(stance(s,sign*w))<.55f*radius) {hit=true;break;}
      }
      separated=separated && hit && observedAir(a) && observedAir(b) &&
          lidar_map_interface_->getDisToOcc(b)>radius;
    }
    // Two independent measured lanes can meet around the end of a
    // separating surface. The separator itself blocks the middle ray.
    if (separated) {
      if (std::abs(far-ahead)>.8f*width) continue;
    } else if (require_separator || !wide || middle<=0.f || std::abs(middle-far)>.6f*width) continue;
    const float barrier=.5f*(far+(separated ? ahead : middle));
    const float r=.5f*width;
    float lead=std::max(0.f,barrier-r-.6f*radius);
    const int n=int(std::ceil(float(M_PI)*r/cell));
    vector<Eigen::Vector3f> route;
    for (;;) {
      route={body};
      for (float s=cell;s<lead;s+=cell) route.push_back(stance(s,0.f));
      for (int i=0;i<=n;++i) {
        const float a=float(M_PI)*i/n;
        route.push_back(stance(lead+r*std::sin(a),sign*r*(1.f-std::cos(a))));
      }
      if (certifyFlightGuide(route)) break;
      route.clear();
      if (lead<=0.f) break;
      lead=std::max(0.f,lead-2.f*cell);
    }
    if (route.empty()) continue;
    const size_t end_turn=route.size()-1;
    float length=0.f,fresh=0.f;
    for (float s=cell;s<=2.5f*trust;s+=cell) {
      const auto q=stance(lead-s,sign*width);
      if (!certifyFlightGuide({route.back(),q})) break;
      route.push_back(q);
      // Two stances on opposite sides of one open strip can see the same
      // surfaces. A measured separating wall still makes the next pass new.
      if (!visitedStance(q) && !nearFlightPath(q,.65f*trust,2.f*trust,.35f*radius)) fresh+=cell;
      length=s;
    }
    if (fresh<.75f*length) continue;
    vector<ObservationSample> exposed;
    if (length<(separated ? trust : 1.5f*trust)) {
      // A separator occludes the neighbouring strip until the vehicle
      // rounds its end. A certified turn plus a short unflown exit is a
      // useful viewpoint if it faces a coherent band of unknown air.
      if (!separated || length<2.f*cell) continue;
      int open_rays=0;
      for (float lateral:{-cell,0.f,cell}) {
        int unknown=0;
        for (int j=1;j<=3;++j) {
          Eigen::Vector3f q=route.back();
          q.head<2>()+=lateral*right-j*cell*along;
          if (!isInBox(q) || lidar_map_interface_->getDisToOcc(q)<=radius) break;
          if (!observedAir(q)) {
            exposed.push_back({ObservationSample::Kind::UnknownAir,q});
            ++unknown;
          }
        }
        if (unknown>=2) ++open_rays;
      }
      if (open_rays<2) continue;
      for (size_t j=1;j<=end_turn;++j) {
        if (!visitedStance(route[j]) && !nearFlightPath(route[j],.65f*trust,2.f*trust,.35f*radius))
          fresh+=(route[j]-route[j-1]).norm();
      }
    }
    if (exposed.empty() && frtp_.target_max_abs_normal_z_<1.f) {
      // A parallel track through already visible air needs something new
      // to observe on its outgoing leg before it competes for first place.
      vector<Eigen::Vector3f> outgoing(route.begin()+end_turn,route.end());
      auto view=evaluateSurveyPath(route[end_turn],-along,std::move(outgoing),fresh,true,false);
      if (!view) continue;
      exposed=std::move(observations_.at(view.get())->contour_support_);
      observations_.erase(view.get());
    }
    auto node=std::make_shared<TopoNode>();
    node->is_viewpoint_=true;node->center_=route.back();node->yaw_=std::atan2(-along.y(),-along.x());
    auto obs=std::make_shared<ClusterInfo>();
    obs->center_=node->center_;obs->normal_=Eigen::Vector3f(along.x(),along.y(),0.f);
    obs->contour_entry_=route[std::min(route.size()-1,end_turn+size_t(std::ceil(radius/cell)))];
    obs->contour_approach_=std::move(route);obs->contour_preview_={node->center_};
    obs->fresh_contour_length_=fresh;
    obs->contour_support_=std::move(exposed);
    observations_[node.get()]=obs;
    return node;
  }
  return {};
}

// Before the first horizontal departure, body yaw alone can point into a fully
// observed pocket. Test the outgoing direction of its wall preview for an
// actual free/unknown boundary before commanding a round trip into the pocket.
TopoNode::Ptr FrontierManager::initialBoundaryObservation(const Eigen::Vector3f &body,const TopoNode::Ptr &view) {
  const auto &wall=*observations_.at(view.get());
  const auto &preview=wall.contour_preview_;
  if (wall.contour_entry_ || preview.size()<2 || !unseenSupport(wall).empty() || followsRightWall(preview)) return {};
  const float radius=graph_->bubble_min_radius_;
  const Eigen::Vector2f approach=(view->center_-body).head<2>();
  const Eigen::Vector2f outgoing=(preview.back()-preview.front()).head<2>();
  if (outgoing.norm()<=radius || approach.norm()<=radius ||
      approach.dot(outgoing)>=-.6f*approach.norm()*outgoing.norm()) return {};
  for (const auto &past:flight_history_)
    if ((past-body).head<2>().squaredNorm()>radius*radius) return {};
  vector<Eigen::Vector3f> route;
  if (!observedBoundaryGuide(body,route,&outgoing,nullptr,std::max(3.f*radius,approach.norm())) ||
      route.size()<2) return {};

  const auto entry=preferRightBranch(body,route,true);
  auto observation=boundaryObservation(std::move(route));
  if (observation && entry) {
    auto &wall=*observations_.at(observation.get());
    wall.right_hand_opening_=true;
    wall.contour_entry_=entry;
  }
  return observation;
}

// Resolve a fork in the local frame of the approach. The same observation
// geometry applies after takeoff and after a transfer to new measured air.
std::optional<Eigen::Vector3f> FrontierManager::preferRightBranch(const Eigen::Vector3f &body,
                                                                 vector<Eigen::Vector3f> &route,bool initial) {
  const float radius=graph_->bubble_min_radius_;
  if (route.size()<2 || std::any_of(route.begin(),route.end(),[&](const auto &point) {
        return std::abs(point.z()-body.z())>radius;
      })) return {};
  const auto lead=std::find_if(route.begin()+1,route.end(),[&](const Eigen::Vector3f &point) {
    return (point-body).head<2>().norm()>=2.f*radius;
  });
  std::optional<Eigen::Vector3f> entry;
  if (lead!=route.end()) {
    Eigen::Vector2f along=(*lead-body).head<2>().normalized();
    if (!initial) {
      const auto travelled=recentFlightDirection(body);
      if (travelled) {
        if (travelled->dot(along)<-.25f) return {};
        along=*travelled;
      }
    }
    const Eigen::Vector2f right(along.y(),-along.x());
    auto pivot=lead;
    for (auto it=lead+1;it!=route.end();++it) {
      if (std::abs((*it-body).head<2>().dot(right))>radius ||
          (*it-*pivot).head<2>().dot(along)<-.5f*radius) break;
      pivot=it;
    }
    const float side=(route.back()-body).head<2>().dot(right);
    // A nearly straight recovery guide already advances the current sweep.
    // Compare arms when the reachable continuation actually bends left.
    if (!initial && side>=-.5f*radius) return {};
    if (side<=radius) {
      vector<Eigen::Vector3f> branch;
      const auto band=currentSurveyBand(body);
      const float depth=std::max(2.f*radius,2.5f*radius-(*pivot-body).head<2>().dot(right));
      const bool found=observedBoundaryGuide(*pivot,branch,&right,nullptr,depth,nullptr,band) ||
          (depth>2.f*radius && observedBoundaryGuide(*pivot,branch,&right,nullptr,2.f*radius,nullptr,band));
      surface_body_=body;
      if (found && branch.size()>1 && (branch.back()-body).head<2>().dot(right)>radius &&
          std::all_of(branch.begin(),branch.end(),[&](const Eigen::Vector3f &point) {
            const auto offset=(point-*pivot).head<2>().eval();
            return offset.dot(right)>=-.5f*radius && std::abs(offset.dot(along))<=2.f*radius;
          })) {
        vector<Eigen::Vector3f> turn(route.begin(),pivot+1);
        turn.insert(turn.end(),branch.begin()+1,branch.end());
        if (certifyFlightGuide(turn)) {
          route.swap(turn);
        }
      }
      // A direct diagonal into the left arm can hide the junction's centre
      // from the tentative pivot. Search for a second connected observation
      // at the same distance along the approach, using its full measured path.
      if (!initial && (route.back()-body).head<2>().dot(right)<=radius) {
        const float forward_extent=(route.back()-body).head<2>().dot(along);
        vector<Eigen::Vector3f> alternative;
        if (forward_extent>2.f*radius &&
            observedBoundaryGuide(body,alternative,&along,nullptr,0.f,nullptr,band,forward_extent) &&
            std::all_of(alternative.begin(),alternative.end(),[&](const Eigen::Vector3f &p) {
              const Eigen::Vector2f offset=(p-body).head<2>();
              return offset.dot(along)>=-radius && offset.dot(right)>=-radius;
            })) route.swap(alternative);
      }
    }
    if ((route.back()-body).head<2>().dot(right)>radius) {
      const auto inside=std::find_if(route.begin(),route.end(),[&](const Eigen::Vector3f &point) {
        return (point-body).head<2>().dot(right)>1.5f*radius;
      });
      entry=inside!=route.end() ? *inside : route.back();
    }
  }
  return entry;
}

TopoNode::Ptr FrontierManager::boundaryObservation(vector<Eigen::Vector3f> route) {
  if (route.size()<2) return {};
  const Eigen::Vector3f goal=route.back();
  auto observation=std::make_shared<ClusterInfo>();
  const Eigen::Vector3i key=(goal/frtp_.cell_size_).array().floor().cast<int>();
  if (!unknownBoundaryFaces(key,&observation->contour_support_)) return {};
  auto node=std::make_shared<TopoNode>();
  node->center_=goal;node->is_viewpoint_=true;
  const Eigen::Vector3f edge=goal-route[route.size()-2];
  const Eigen::Vector3f chord=goal-route.front();
  node->yaw_=edge.head<2>().squaredNorm()>.01f ? std::atan2(edge.y(),edge.x()) : std::atan2(chord.y(),chord.x());
  float fresh=0.f;
  for (size_t i=route.size()-1;i>0 && fresh<frtp_.good_observation_trust_length_;--i) {
    const Eigen::Vector3f leg=route[i-1]-route[i];
    const int n=std::max(1,int(std::ceil(leg.norm()/frtp_.cell_size_)));
    bool flown=false;
    for (int k=0;k<n;++k) {
      if (visitedStance(route[i]+(float(k)/n)*leg)) {flown=true;break;}
      fresh+=leg.norm()/n;
    }
    if (flown) break;
  }
  observation->fresh_contour_length_=std::min(fresh,frtp_.good_observation_trust_length_);
  observation->center_=goal;
  observation->normal_=Eigen::Vector3f(-std::cos(node->yaw_),-std::sin(node->yaw_),0.f);
  observation->contour_approach_=std::move(route);
  observation->contour_preview_={goal};
  observations_[node.get()]=std::move(observation);
  return node;
}

void FrontierManager::appendSurfaceViewpoints(const Eigen::Vector3f &body,vector<TopoNode::Ptr> &viewpoints,bool include_alternative) {
  vector<TopoNode::Ptr> next;
  appendContourViewpoints(body,next,include_alternative);
  const float radius=graph_->bubble_min_radius_;
  // Classify the actual approach to a measured wall. The endpoint on the
  // far side of a wall can lie to the right even if its approach starts
  // back out through the entrance.
  if (auto heading=recentFlightDirection(body)) {
    const Eigen::Vector2f right(heading->y(),-heading->x());
    const auto opening=std::find_if(next.begin(),next.end(),[&](const TopoNode::Ptr &node) {
      auto &view=*observations_.at(node.get());
      if (view.fresh_contour_length_<radius || unseenSupport(view).empty() ||
          std::abs(node->center_.z()-body.z())>radius ||
          (node->center_-body).head<2>().dot(right)<radius) return false;
      // A surveyed corner can lie beyond the first few metres of straight
      // approach. Follow the measured outline to its first right-hand bend
      // while it keeps returning previously unseen wall faces.
      const bool attached=perimeter_target_ && (node->center_-*perimeter_target_).norm()<.05f;
      const bool measured_corner=attached && !view.contour_entry_ &&
          view.fresh_contour_length_>=2.f*radius &&
          std::count_if(view.contour_support_.begin(),view.contour_support_.end(),[&](const auto &sample) {
            return sample.kind==ObservationSample::Kind::WallFace && !observedFace(sample.point,sample.toward_sensor);
          })>=3;
      const float horizon=frtp_.good_observation_trust_length_*(measured_corner ? 2.f : 1.f)+radius;
      float travelled=0.f;
      Eigen::Vector3f previous=body;
      for (const auto &point:view.contour_approach_) {
        travelled+=(point-previous).norm();
        previous=point;
        const Eigen::Vector2f delta=(point-body).head<2>();
        if (travelled>horizon ||
            delta.dot(*heading)<-radius || std::abs(point.z()-body.z())>radius) break;
        if (delta.dot(right)>1.5f*radius && delta.dot(right)>.45f*delta.norm() && !visitedStance(point)) {
          view.right_hand_opening_=true;
          if (measured_corner) view.contour_entry_=point;
          return true;
        }
      }
      return false;
    });
    if (opening!=next.end()) {
      std::rotate(next.begin(),opening,opening+1);
      perimeter_target_=next.front()->center_;
    }
  }
  const bool right_opening=!next.empty() && observations_.at(next.front().get())->right_hand_opening_;
  bool continuing_wall=false,backtracks=false,long_pass=false,occluded_turn=false;
  bool crosses_left=false;
  Eigen::Vector2f travel_direction=Eigen::Vector2f::Zero();
  if (!next.empty()) {
    const auto &wall=*observations_.at(next.front().get());
    const Eigen::Vector3f target=next.front()->center_;
    const bool attached=perimeter_target_ && (*perimeter_target_-target).norm()<.05f;
    Eigen::Vector2f heading(std::cos(graph_->odom_node_->yaw_),std::sin(graph_->odom_node_->yaw_));
    for (auto it=flight_history_.rbegin();it!=flight_history_.rend();++it) {
      const Eigen::Vector2f travel=(body-*it).head<2>();
      if (travel.norm()>2.f*radius) {heading=travel.normalized();break;}
    }
    travel_direction=heading;
    // A surface sample on the far left can lead around an obstacle while
    // missing a still-open, uninspected strip directly ahead of the body.
    const Eigen::Vector2f left(-heading.y(),heading.x());
    const Eigen::Vector2f offset=(target-body).head<2>();
    crosses_left=!wall.contour_entry_ && offset.dot(left)>2.f*radius &&
        offset.dot(heading)<.75f*offset.dot(left) &&
        (wall.contour_preview_.empty() || (wall.contour_preview_.back()-body).head<2>().dot(left)>radius);
    float arc=0.f;Eigen::Vector3f past=body;
    for (auto it=flight_history_.rbegin();it!=flight_history_.rend() && arc<2.f*frtp_.good_observation_trust_length_;++it) {
      arc+=(*it-past).norm();past=*it;
    }
    long_pass=arc>=2.f*frtp_.good_observation_trust_length_ && (body-past).head<2>().dot(heading)>.85f*arc;
    backtracks=long_pass && (target-body).head<2>().dot(heading)<-2.f*radius;
    // Distance to an unrelated surface is transit. It cannot establish a
    // continuing wall when a productive measured swath lies straight ahead.
    continuing_wall=attached && (wall.fresh_contour_length_>=2.f*radius || wall.contour_entry_ ||
        ((wall.close_surface_sample_ || !wall.contour_support_.empty()) &&
         (target-body).head<2>().dot(heading)>4.f*radius));
    // Going around the front of an isolated obstruction can put the preview
    // back beside the incoming leg. An opposing wall or shelf makes the
    // cross-connection fail, so neighbouring real passages remain distinct.
    const auto &in=wall.contour_approach_;
    const auto &out=wall.contour_preview_;
    // An outgoing right-hand edge on the other side of a measured wall
    // stays attached while rounding its end, even when the inside face
    // has already been inspected from the approach.
    if (attached && out.size()>1 && wall.fresh_contour_length_>=frtp_.cell_size_) {
      const Eigen::Vector2f approach=(target-body).head<2>();
      const Eigen::Vector2f onward=(out.back()-target).head<2>();
      const Eigen::Vector3f chord=out.back()-body;
      if (approach.norm()>2.f*radius && onward.norm()>2.f*radius &&
          approach.dot(onward)<-.35f*approach.norm()*onward.norm() &&
          approach.x()*onward.y()-approach.y()*onward.x()<-.3f*approach.norm()*onward.norm() &&
          std::abs(chord.z())<radius) {
        const int steps=int(std::ceil(chord.norm()/(.5f*frtp_.cell_size_)));
        for (int j=1;j<steps && !occluded_turn;++j) {
          const Eigen::Vector3f probe=body+chord*(float(j)/steps);
          occluded_turn=lidar_map_interface_->getDisToOcc(probe)<.5f*frtp_.cell_size_;
        }
      }
      continuing_wall=continuing_wall || occluded_turn;
    }
    if (continuing_wall && !occluded_turn && !wall.contour_entry_ && in.size()>1 && out.size()>1) {
      const Eigen::Vector3f axis=in.back()-body;
      if (axis.norm()>2.f*radius) {
        const Eigen::Vector3f direction=axis.normalized();
        for (const auto &q:out) {
          if ((in.back()-q).dot(direction)<2.f*radius) continue;
          for (size_t j=1;j<in.size();++j) {
            const Eigen::Vector3f leg=in[j]-in[j-1];
            const float fraction=std::clamp((q-in[j-1]).dot(leg)/std::max(.0001f,leg.squaredNorm()),0.f,1.f);
            const Eigen::Vector3f near=in[j-1]+fraction*leg;
            const float distance=(near-q).norm();
            if (distance<2.f*radius && (distance<.05f || certifyFlightGuide({near,q}))) {
              continuing_wall=false;
              break;
            }
          }
          if (!continuing_wall) break;
        }
      }
    }
  }
  auto swath=(right_opening || occluded_turn) ? TopoNode::Ptr{} : parallelObservation(body,continuing_wall);
  if (swath) {
    // Three short blocked rays can straddle an opening. Before reversing,
    // check whether a productive measured path actually crosses the line
    // where the proposed turn would reverse its longitudinal velocity.
    const Eigen::Vector2f along(-std::cos(swath->yaw_),-std::sin(swath->yaw_));
    float turn_extent=0.f;
    for (const auto &point:observations_.at(swath.get())->contour_approach_)
      turn_extent=std::max(turn_extent,(point-body).head<2>().dot(along));
    auto through=forwardObservation(body);
    if (through && (through->center_-body).head<2>().dot(along)>
                    turn_extent+frtp_.good_observation_trust_length_) {
      observations_.erase(swath.get());
      swath=through;
    } else if (through) observations_.erase(through.get());
  }
  // At the end of a sustained pass the right face may end beside open
  // floor. Continue across a wide, uninspected strip. A narrow continuation
  // keeps the primary right-hand turn into a newly measured opening.
  if (!swath && !right_opening && !occluded_turn && (!continuing_wall || long_pass))
    swath=forwardObservation(body,continuing_wall && !backtracks);
  // Near the end of the issued guide, look along its outgoing tangent.
  // Body motion still contains the incoming turn. A distant outline can
  // otherwise win by sending the aircraft back through the same entrance.
  const auto continue_wall=[&]() -> TopoNode::Ptr {
    if (right_opening || occluded_turn || previous_outline_.size()<2 || !survey_height_) return {};
    const float trust=frtp_.good_observation_trust_length_,cell=frtp_.cell_size_;
    const auto &tip=previous_outline_.back();
    const auto tail=std::find_if(previous_outline_.rbegin()+1,previous_outline_.rend(),[&](const auto &p) {
      return (tip-p).template head<2>().norm()>=2.f*radius;
    });
    const auto motion=recentFlightDirection(body);
    if (!motion || tail==previous_outline_.rend() || std::abs(tip.z()-body.z())>radius) return {};
    const Eigen::Vector2f axis=(tip-*tail).head<2>().normalized(),right(axis.y(),-axis.x());
    const Eigen::Vector2f offset=(tip-body).head<2>();
    const float ahead=offset.dot(axis);
    if (motion->dot(axis)<.75f || ahead<0.f || ahead>trust || std::abs(offset.dot(right))>radius) return {};
    vector<Eigen::Vector3f> route{body};
    size_t supported=0;
    float gap=0.f;
    for (float along=cell;along<=frtp_.update_length_;along+=cell) {
      Eigen::Vector3f q=body;
      q.head<2>()+=along*axis;
      q.z()+=std::clamp(*survey_height_-body.z(),-.12f*along,.12f*along);
      if (!isInBox(q) || !certifyFlightGuide({route.back(),q})) break;
      bool wall=false;
      for (float w=radius;w<=trust;w+=cell) {
        Eigen::Vector3f side=q;side.head<2>()+=w*right;
        if (lidar_map_interface_->getDisToOcc(side)<cell) {wall=true;break;}
      }
      gap=wall ? 0.f : gap+cell;
      if (gap>radius) break;
      route.push_back(q);
      if (wall) supported=route.size();
    }
    if (supported<2 || (route[supported-1]-body).head<2>().dot(axis)<ahead+2.f*radius) return {};
    route.resize(supported);
    auto candidate=evaluateSurveyPath(body,axis,std::move(route),0.f,true,false);
    if (!candidate) return {};
    auto &obs=*observations_.at(candidate.get());
    const auto crosses=std::find_if(obs.contour_approach_.begin(),obs.contour_approach_.end(),[&](const auto &p) {
      return (p-body).template head<2>().dot(axis)>ahead+radius && !visitedStance(p);
    });
    if (crosses==obs.contour_approach_.end()) {observations_.erase(candidate.get());return {};}
    obs.contour_entry_=*crosses;
    return candidate;
  };
  if (auto onward=continue_wall()) {
    if (swath) observations_.erase(swath.get());
    swath=onward;
  }
  if (swath) {
    // A measured free-space pass is a tour alternative. Keep the wall
    // available to the first-edge comparison at the same timestamp.
    if (!include_alternative) {
      for (const auto &v:next) observations_.erase(v.get());
      next.clear();
    }
    if (next.empty()) perimeter_target_=swath->center_;
    next.push_back(swath);
  } else if (include_alternative && !right_opening && continuing_wall && !occluded_turn && !long_pass && crosses_left) {
    auto &wall=*observations_.at(next.front().get());
    if (auto ahead=forwardObservation(body,false)) {
      const auto &observation=*observations_.at(ahead.get());
      const Eigen::Vector2f offset=(ahead->center_-body).head<2>();
      if (offset.dot(travel_direction)>.65f*offset.norm() &&
          (ahead->center_-next.front()->center_).norm()>2.f*radius &&
          unseenSupport(observation).size()>unseenSupport(wall).size()) {
        next.push_back(ahead);
        // Both departures now have measured support. Let the tour price
        // them without binding its first edge to the left-hand sample.
      } else observations_.erase(ahead.get());
    }
  }
  if (!right_opening && next.size()==1 && previous_outline_.empty()) {
    if (auto boundary=initialBoundaryObservation(body,next.front())) {
      if (include_alternative) next.push_back(boundary);
      else {
        observations_.erase(next.front().get());
        next.front()=boundary;
        perimeter_target_=boundary->center_;
      }
    }
  }
  viewpoints.insert(viewpoints.end(),next.begin(),next.end());
}

void FrontierManager::updateFrontierClusters(
    vector<ClusterInfo::Ptr> &cluster_updated, vector<int> &cluster_removed) {
  if (surface_continuity_) updateSurfaceSamples();
  PointVector frt_new;
  bool trust_close_fov_edge;
  nh_.param("FrontierManager/trust_close_fov_edge",trust_close_fov_edge,surface_continuity_);
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

  omp_set_num_threads(epic_ros::planningWorkerCount(4));
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
    if (get_state(cell) == SPARSE && has_dense_nbr(cell)) {
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
  omp_set_num_threads(epic_ros::planningWorkerCount(4));
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
  bool measured_surface=false;
  nh_.param("FrontierManager/measured_surface_visibility",measured_surface,surface_continuity_);
  vector<Eigen::Vector3f> target_surface(cluster->cells_.size(),Eigen::Vector3f::Zero());
  vector<bool> target_supported(cluster->cells_.size(),false);
  vector<bool> remaining(cluster->cells_.size(),true);
  double support_distance;
  // Scale the acceptable nearest return with the frontier voxel.
  nh_.param("FrontierManager/max_frontier_support_distance",support_distance,1.375*frtp_.cell_size_);
  if(measured_surface)for(size_t j=0;j<cluster->cells_.size();++j) {
    PointVector nearest;vector<float> distances;
    lidar_map_interface_->KNN(cluster->cells_[j],1,nearest,distances);
    if(!nearest.empty() && (!surface_continuity_ || distances.front()<=support_distance*support_distance)) {
      target_surface[j]=nearest[0].getVector3fMap();target_supported[j]=true;
    }
    if (surface_continuity_) {
      const auto &normal=cluster->norms_[j];
      remaining[j]=target_supported[j] && !observedFace(cluster->cells_[j].getVector3fMap(),normal) &&
          !observedFace(target_surface[j],normal) &&
          inspectionSurface(*lidar_map_interface_,nearest.front(),frtp_.cell_size_,frtp_.target_max_abs_normal_z_);
    }
  }
  unordered_map<Eigen::Vector3i,Eigen::Vector3f,Vector3i_Hash> blocker_surface;
  for (int i = 0; i < vps.size(); i++) {
    Eigen::Vector3f vp = vps[i].getVector3fMap();
    for (int j = 0; j < cluster->cells_.size(); j++) {
      if (!remaining[j]) continue;
      Eigen::Vector3f frt = cluster->cells_[j].getVector3fMap();
      Eigen::Vector3f dir = (frt - vp).cast<float>();
      float distance = dir.norm();
      dir.normalize();
      if (distance > frtp_.good_observation_trust_length_)
        continue;
      CELL_STATE state = get_state(cluster->cells_[j]);
      if (state == FRONTIER_DIR &&
          distance > frtp_.good_observation_force_trust_length_)
        continue;
      Eigen::Vector3f norm = cluster->norms_[j];
      float sin_theta = dir.dot(norm);
      if (measured_surface) {
        sin_theta = -sin_theta;
        if (sin_theta <= 0.f) continue;
      }
      float cos_thera = sqrt(std::max(0.f, 1 - sin_theta * sin_theta));
      float delta = M_PI / 100.0;
      float score = sin_theta / (sin_theta + delta * cos_thera);
      if (score < frtp_.good_observation_direction_score_)
        continue;
      ray_caster.input(frt.cast<double>(), vp.cast<double>());
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
            continue;
          }
        }
        if (!lidar_map_interface_->IsInBox(pt) || state == DENSE ||
            state == SPARSE) {
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
    int max_yaw_idx = distance(yaw_score.begin(),std::max_element(yaw_score.begin(),yaw_score.end()));
    if (surface_continuity_) {
      const float body_yaw=graph_->odom_node_->yaw_;
      const int best=yaw_score[max_yaw_idx];
      double least_turn=std::numeric_limits<double>::infinity();
      for (int k=0;k<8;++k) {
        if (yaw_score[k] < std::max(1.,.95*best)) continue;
        const double angle=(45.*(k-4)+22.5)*M_PI/180.;
        const double turn=std::abs(std::remainder(angle-body_yaw,2.*M_PI));
        if (turn<least_turn) {least_turn=turn;max_yaw_idx=k;}
      }
    }
    // if (yaw_score[max_yaw_idx] == 0)
    //   continue;
    // Prefer a well-conditioned, front-facing measurement among positions
    // with comparable visible support. Keep the actual observable area primary.
    const Eigen::Vector3f offset = vp - cluster->center_;
    const float range = offset.norm();
    const float facing = range > .01f ? std::clamp(offset.dot(cluster->normal_) / range, 0.f, 1.f) : 0.f;
    const float body_z=graph_->odom_node_->center_.z();
    const float cruise_z=survey_height_.value_or(
        std::abs(body_z-vpp_.fixed_height_)<frtp_.good_observation_force_trust_length_ ? vpp_.fixed_height_ : body_z);
    const float altitude_change = std::abs(vp.z() - cruise_z);
    score[i] = yaw_score[max_yaw_idx];
    if (surface_continuity_ && std::isfinite(facing)) score[i] *= (.7f + .3f * facing) /
        (1.f + .8f * altitude_change + .1f * std::abs(range - frtp_.good_observation_force_trust_length_));

    // Eigen::Vector4f hs = cluster->view_halfspace_;
    // Eigen::Vector4f vp_h(vp.x(), vp.y(), vp.z(), 1.0);
    // if (vp_h.dot(hs) < -0.1) {
    //   score[i] *= (1.5 - vp_h.dot(hs));
    // }
    yaw[i] = (45.0 * (max_yaw_idx - 4) + 22.5) / 180.0 * M_PI;
  }
  // An external position follower completes an in-tolerance translation before
  // it can realize the observation yaw. Prefer another visible stance in that case.
  bool external = false;
  nh_.param("fsm/external_local_planner", external, false);
  if (external) {
    for (size_t i = 0; i < vps.size(); ++i) {
      const double yaw_error = std::remainder(yaw[i] - graph_->odom_node_->yaw_, 2. * M_PI);
      if ((vps[i].getVector3fMap() - graph_->odom_node_->center_).norm() < vpp_.reached_position_tolerance_ &&
          std::abs(yaw_error) >= vpp_.reached_yaw_tolerance_) score[i] = 0.f;
    }
  }
  int best_vp_idx = std::distance(score.begin(),
                                  std::max_element(score.begin(), score.end()));
  if (score[best_vp_idx] == 0) {
    cluster->is_reachable_ = false;
    cluster->vp_clusters_.clear();
  } else {
    cluster->is_reachable_ = true;
    cluster->visible_cells_.clear();
    const auto chosen = vps[best_vp_idx].getVector3fMap();
    const auto view_rotation = Eigen::AngleAxisf(-vpp_.lidar_pitch_ * float(M_PI / 180.), Eigen::Vector3f::UnitY()) *
                               Eigen::AngleAxisf(-yaw[best_vp_idx], Eigen::Vector3f::UnitZ());
    for (const auto &cell : occ_free_frts[best_vp_idx]) {
      const Eigen::Vector3f ray = view_rotation * (cell.getVector3fMap() - chosen);
      const float pitch = std::atan2(ray.z(), ray.head<2>().norm());
      if (pitch >= vpp_.fov_down_ && pitch <= vpp_.fov_up_) cluster->visible_cells_.push_back(cell);
    }
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
  vector<Eigen::Vector3f> level_stances,face_stances;
  if (surface_continuity_ && cluster->normal_.head<2>().norm()>.5f) {
    const Eigen::Vector3f normal=cluster->normal_;
    const Eigen::Vector3f tangent(normal.y(),-normal.x(),0.f);
    const float dz=survey_height_.value_or(graph_->odom_node_->center_.z())-cluster->center_.z();
    if (!vpp_.fixed_height_enabled_ && std::abs(dz)>frtp_.cell_size_) {
      // Observe a raised or lowered patch from the present flight band if
      // the measured air and sensor FOV allow it. Test both stances below.
      const Eigen::Vector2f outward=normal.head<2>().normalized();
      const float pitch=vpp_.lidar_pitch_*float(M_PI/180.);
      const float aperture=dz<0.f ? vpp_.fov_up_-pitch : pitch-vpp_.fov_down_;
      if (aperture>.12f) {
        const float reach=std::max<float>(2.f*graph_->bubble_min_radius_,
            std::abs(dz)/std::tan(std::min(1.4f,.8f*aperture)));
        if (reach<=frtp_.good_observation_trust_length_*1.5f) {
          for (float side:{0.f,-1.f,1.f}) {
            Eigen::Vector3f offset=side*graph_->bubble_min_radius_*tangent;
            offset.head<2>()+=reach*outward;
            offset.z()=dz;
            level_stances.push_back(offset);
          }
        }
      }
    }
    for (float side:{0.f,-1.f,1.f})
      face_stances.push_back(graph_->bubble_min_radius_*(2.f*normal+side*tangent));
    // The preferred standoff can lie against the opposite wall at a blind
    // end. Try the interior of the measured safe band before dropping the face.
    const float near=measuredStanceClearance()+.5f*frtp_.cell_size_;
    for (float side:{0.f,-.5f,.5f})
      face_stances.push_back(near*normal+side*graph_->bubble_min_radius_*tangent);
  }
  const auto collect=[&](const vector<Eigen::Vector3f> &offsets,bool nearest_face) {
    for (const auto &ovp : offsets) {
      if (!nearest_face && !face_stances.empty() && ovp.dot(cluster->normal_)<graph_->bubble_min_radius_) continue;
      Eigen::Vector3f vp = ovp + cluster->center_;
      if (vpp_.fixed_height_enabled_) vp.z() = vpp_.fixed_height_;
      else if (surface_continuity_ && std::abs(ovp.z())<frtp_.cell_size_ &&
               std::abs(cluster->center_.z()-vpp_.fixed_height_)<frtp_.good_observation_force_trust_length_)
        vp.z()=vpp_.fixed_height_;
      if (surface_continuity_ && (!withinFlightHeight(vp) || !observedAir(vp))) continue;
      if (lidar_map_interface_->getDisToOcc(vp) < (surface_continuity_ ? measuredStanceClearance() : std::max<float>(.9f,graph_->bubble_min_radius_)))
        continue;
      if (!isInBox(vp))
        continue;
      Eigen::Vector3i idx;
      graph_->getIndex(vp, idx);
      if (graph_->getRegionNode(idx) == nullptr)
        continue;
      // The air voxel may have been filled after the vehicle passed it. Check
      // the actual past flight on this side before scheduling a second visit.
      // Keep a similar stance if it now reveals a largely unmeasured face.
      const bool passed=surface_continuity_ && visitedStance(vp);
      if (surface_continuity_ && (passed || nearFlightPath(vp,1.4f*graph_->bubble_min_radius_,0.f,.2f))) {
        size_t seen=0;
        for (const auto &cell:cluster->cells_) {
          const Eigen::Vector3f ray=vp-cell.getVector3fMap();
          if (ray.squaredNorm()>.001f && observedFace(cell.getVector3fMap(),ray.normalized())) ++seen;
        }
        if (seen>0 && 2*seen>=cluster->cells_.size()) {
          continue; // Reject this stance; other views may still expose the face.
        }
      }
      vps_init.emplace_back(vp.x(), vp.y(), vp.z());
      if (nearest_face) break;
    }
  };
  if (!level_stances.empty()) collect(level_stances,true);
  if (!face_stances.empty()) collect(face_stances,true);
  if (vps_init.empty()) collect(origin_viewpoints_,false);
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
  graph_->insertNodes(nodes2insert, true); // only_raycast=true 可以显著加速
  epic_ros::Time t2 = epic_ros::Time::now();
  vector<bool> vp_cluster_kept;
  vp_cluster_kept.resize(nodes2insert.size(), true);
  // 可以并行
  for (int i = 0; i < nodes2insert.size(); i++) {
    if (nodes2insert[i]->neighbors_.empty()) {
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
    if (!graph_->graphSearch(closest_node, nodes2insert[i], topo_path, 3e-4)) {
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

bool FrontierManager::approachedClusterCells(const ClusterInfo &cluster,std::optional<float> survey_band,bool require_face) const {
  const float trust=frtp_.good_observation_trust_length_,radius=graph_->bubble_min_radius_;
  const float cell_size=frtp_.cell_size_,edge_range=trust+std::sqrt(3.f)*cell_size;
  const float face_range=std::max(trust,std::min(2.f*trust,float(frtp_.update_length_)));
  const bool closed_perimeter=perimeter_pass_ && perimeter_pass_->finished;
  if (!survey_band && closed_perimeter) survey_band=perimeter_pass_->level;
  const bool low_edge=survey_band && !cluster.is_reachable_ && cluster.vp_clusters_.empty() &&
      cluster.last_viewpoint_attempt_>=0. && cluster.box_max_.z()<*survey_band-.5f*radius;
  return !cluster.cells_.empty() && std::all_of(cluster.cells_.begin(),cluster.cells_.end(),[&](const auto &cell) {
    const Eigen::Vector3f point=cell.getVector3fMap();
    const Eigen::Vector3f *nearest=nullptr,*wall_observer=nullptr;
    float nearest_xy=trust*trust,wall_distance=face_range*face_range;
    for (const auto &past:flight_history_) {
      const Eigen::Vector3f offset=point-past;
      const float range2=offset.squaredNorm();
      if (low_edge && range2<wall_distance && std::abs(past.z()-*survey_band)<radius) {
        wall_observer=&past;wall_distance=range2;
      }
      if (!require_face && range2<=trust*trust) return true;
      // Multi-scan face evidence has a longer measured range than a single
      // close pass. Use the same bound as recordFaceHit and require evidence
      // on the side facing this flown position, including at convex corners.
      if ((closed_perimeter || require_face) && range2<=face_range*face_range) {
        const Eigen::Vector3f toward=-offset.normalized();
        const Eigen::Vector3f face=point+cell_size*toward;
        if (observedFace(point,toward) || observedFace(face,toward)) return true;
        if (!require_face && range2<=edge_range*edge_range) {
          vector<Eigen::Vector3f> sight{past,point+2.f*cell_size*toward};
          if ((observedGuide(sight) && graph_->parallel_bubble_astar_->collisionCheck_shortenPath(sight,.2f*cell_size)) ||
              observedFace(sight.back(),toward)) return true;
        }
      }
      if (survey_band && std::abs(past.z()-*survey_band)<=radius && offset.head<2>().squaredNorm()<nearest_xy) {
        nearest=&past;
        nearest_xy=offset.head<2>().squaredNorm();
      }
    }
    // The lowest return on a continuous wall can be too sparse to retire its
    // individual voxel. Require returns from this edge up to an inspected
    // patch of the same wall, on the side of a previously flown observer.
    if (wall_observer) {
      Eigen::Vector3f side=*wall_observer-point;
      side.z()=0.f;
      if (side.squaredNorm()>.01f) {
        side.normalize();
        const float width=1.125f*cell_size;
        const float top=wall_observer->z()+.5f*cell_size;
        PointVector hits;
        lidar_map_interface_->boxSearch(point-Eigen::Vector3f(width,width,.5f*cell_size),
            Eigen::Vector3f(point.x()+width,point.y()+width,top),hits);
        vector<float> heights;
        float inspected=top+cell_size;
        for (const auto &hit:hits) {
          const Eigen::Vector3f q=hit.getVector3fMap();
          if (!inspectionSurface(*lidar_map_interface_,hit,cell_size,frtp_.target_max_abs_normal_z_)) continue;
          heights.push_back(q.z());
          if (observedFace(q,side)) inspected=std::min(inspected,q.z());
        }
        std::sort(heights.begin(),heights.end());
        float connected=point.z();
        for (const float h:heights) {
          if (h>connected+1.75f*cell_size) break;
          connected=std::max(connected,h);
          if (connected>=inspected) return true;
        }
      }
    }
    // A registration edge below the slab can lie outside the 3-D sensor
    // radius even after a close horizontal pass. Require measured supporting
    // ground on that pass before treating such an unsampleable cell as spent.
    if (require_face || !nearest || point.z()>=*survey_band-ground_clearance_) return false;
    const auto floor=measuredSurveyHeight(*nearest,ground_clearance_);
    return floor && std::abs(*floor-*survey_band)<=radius && point.z()<*floor-ground_clearance_;
  });
}

// Unvisited air above an already flown floor is a poor reason to reopen a
// completed survey. Require positive floor evidence and a sampling attempt
// at every remaining surface. An untested or distant surface keeps the tour open.
bool FrontierManager::exhaustedAboveFlownFloor(const vector<Eigen::Vector3f> &route) const {
  if (!surface_continuity_ || ground_clearance_<=0. || route.size()<2 || flight_history_.size()<2)
    return false;
  const float radius=graph_->bubble_min_radius_,trust=frtp_.good_observation_trust_length_;
  const auto band=currentSurveyBand(route.front());
  const auto &goal=route.back();
  if (!band || goal.z()<=*band+radius || !flownObservationStance(route.front())) return false;
  const auto floor=measuredSurveyHeight(goal,ground_clearance_);
  if (!floor || std::abs(*floor-*band)>radius || followsRisingFloor(route)) return false;
  const Eigen::Vector3f lower(goal.x(),goal.y(),*floor);
  if (!nearFlightPath(lower,trust,0.f,.2f*frtp_.cell_size_,radius)) return false;
  for (const auto &c:cluster_list_) {
    if (!c || c->is_dormant_ || observedCluster(*c)) continue;
    if (c->last_viewpoint_attempt_<0. || c->is_reachable_ || !c->vp_clusters_.empty() ||
        !approachedClusterCells(*c,band)) return false;
  }
  return true;
}

// A high label can remain even when its wall face has already been scanned
// from below. Check the actual observer, its measured floor and the same
// face. A route supported by rising ground still warrants exploration.
bool FrontierManager::observedAboveFlownFloor(const ClusterInfo &cluster,float band) const {
  const float radius=graph_->bubble_min_radius_,trust=frtp_.good_observation_trust_length_;
  if (cluster.box_min_.z()<=band+radius || cluster.cells_.empty() ||
      cluster.is_reachable_ || !cluster.vp_clusters_.empty() || cluster.last_viewpoint_attempt_<0.) return false;
  const Eigen::Vector3f point=cluster.center_;
  const Eigen::Vector3f *observer=nullptr;
  float distance=trust*trust;
  for (const auto &past:flight_history_) {
    const float candidate=(past-point).squaredNorm();
    if (std::abs(past.z()-band)<=radius && candidate<distance) {
      observer=&past;distance=candidate;
    }
  }
  if (!observer) return false;
  const auto floor=measuredSurveyHeight(*observer,ground_clearance_);
  if (!floor || std::abs(*floor-band)>radius || followsRisingFloor({*observer,point})) return false;
  const Eigen::Vector3f side=(*observer-point).normalized();
  const Eigen::Vector3f face=point+2.f*frtp_.cell_size_*side;
  return (observedFace(point,side) || observedFace(face,side)) && observedGuide({*observer,face});
}

// A sampled stance can remain in the topology below a completed ascent.
// Verify every cell on the side that the stance was meant to inspect.
bool FrontierManager::surveyedLowerSurface(const ClusterInfo &cluster,const Eigen::Vector3f &body) const {
  const float radius=graph_->bubble_min_radius_,cell=frtp_.cell_size_;
  if (cluster.cells_.empty() || cluster.center_.z()>=body.z()-radius ||
      cluster.last_viewpoint_attempt_<0. || !cluster.is_reachable_ || cluster.vp_clusters_.empty()) return false;
  const float trust=frtp_.good_observation_trust_length_;
  for (const auto &group:cluster.vp_clusters_) {
    if (group.vps_.empty()) return false;
    for (size_t i=0;i<cluster.cells_.size();++i) {
      const Eigen::Vector3f point=cluster.cells_[i].getVector3fMap();
      const Eigen::Vector3f side=group.center_-point;
      const Eigen::Vector2f normal=(i<cluster.norms_.size() ? cluster.norms_[i] : cluster.normal_).head<2>();
      if (side.head<2>().norm()<cell) return false;
      if (observedFace(point,side.normalized())) continue;
      const bool seen=std::any_of(flight_history_.begin(),flight_history_.end(),[&](const auto &past) {
        const Eigen::Vector3f ray=past-point;
        if (ray.squaredNorm()>trust*trust || ray.head<2>().norm()<cell ||
            normal.dot(side.head<2>())*normal.dot(ray.head<2>())<=0.f) return false;
        const Eigen::Vector3f toward=ray.normalized();
        vector<Eigen::Vector3f> sight{past,point+cell*toward};
        return observedGuide(sight) && graph_->parallel_bubble_astar_->collisionCheck_shortenPath(sight,.2f*cell);
      });
      if (!seen) return false;
    }
  }
  return true;
}

// A small measured patch on an earlier pass can retain a native frontier
// label after all its viewpoints have failed. Keep the label for future
// replans. Once all remaining patches are of this kind, finish only after
// checking the full set of measured recovery anchors from the current pose.
bool FrontierManager::hasPendingSurveyFrontiers(const Eigen::Vector3f &body) {
  pending_survey_location_.reset();
  if (!surface_continuity_ || ground_clearance_<=0.) return hasInspectionFrontiers();
  const float radius=graph_->bubble_min_radius_,trust=frtp_.good_observation_trust_length_;
  const float lower=body.z()-std::max(2.f*radius,float(ground_clearance_)+frtp_.cell_size_);
  const bool trace=std::getenv("EPIC_TRACE_WALL_CHOICE");
  if (untested_ascent_) {
    if (trace) ROS_WARN_THROTTLE(3., "Survey completion waiting for an untested rising approach");
    return true;
  }
  const auto retired=retiredSurveyCeiling();
  const auto band=currentSurveyBand(body);
  size_t residual=0;
  for (const auto &c:cluster_list_) {
    if (!c || c->is_dormant_ || observedCluster(*c)) continue;
    if (retired && c->box_max_.z()<*retired && c->box_max_.z()<lower) continue;
    if (surveyedLowerSurface(*c,body)) continue;
    // A failed same-level surface is eligible for the final anchor sweep only
    // when every cell was approached and sampling produced zero stances.
    // A higher patch, including a possible new landing, stays pending.
    const bool spent_above=c->box_max_.z()>body.z()+radius && band && flownObservationStance(body) &&
        observedAboveFlownFloor(*c,*band);
    const bool unsampleable_here=frtp_.target_max_abs_normal_z_<1.f &&
        (c->box_max_.z()<=body.z()+radius || spent_above) && !c->is_reachable_ && c->vp_clusters_.empty();
    const bool sampled_current_level=c->is_reachable_ && !c->vp_clusters_.empty() &&
        std::all_of(c->vp_clusters_.begin(),c->vp_clusters_.end(),[&](const auto &group) {
          return !group.vps_.empty() && std::all_of(group.vps_.begin(),group.vps_.end(),[&](const auto &stance) {
            return std::abs(stance.z-body.z())<=radius;
          });
        });
    // A group remains reachable in the topology after the vehicle has flown
    // every sampled stance. Revisit it only if those stances still expose a
    // real wall patch. Test the entire raw surface, including unselected cells.
    const bool visited_group=sampled_current_level && std::all_of(c->vp_clusters_.begin(),c->vp_clusters_.end(),
        [&](const auto &group) {
          return !group.vps_.empty() && std::all_of(group.vps_.begin(),group.vps_.end(),[&](const auto &v) {
            return visitedStance(v.getVector3fMap());
          });
        });
    bool flown_without_gain=false;
    if (visited_group) {
      ClusterInfo surface;
      surface.visible_cells_=c->cells_;
      flown_without_gain=std::none_of(c->vp_clusters_.begin(),c->vp_clusters_.end(),[&](const auto &group) {
        return std::any_of(group.vps_.begin(),group.vps_.end(),[&](const auto &v) {
          return hasRevisitableSurface(surface,v.getVector3fMap());
        });
      });
    }
    const bool spent_stances=flown_without_gain || (sampled_current_level && spentSurveyObservation(*c,c->best_vp_) &&
        std::all_of(c->vp_clusters_.begin(),c->vp_clusters_.end(),[&](const auto &group) {
          return surveyedEnclosureStance(group.center_);
        }));
    const bool extended=(c->box_max_-c->box_min_).maxCoeff()>trust;
    // A single long label can span many independently inspected wall cells.
    // Its extent alone does not prove any cell still needs inspection.
    const bool extended_without_evidence=extended &&
        (!unsampleable_here || !approachedClusterCells(*c,band,true));
    if ((c->box_max_.z()>=lower && !unsampleable_here && !spent_stances) || extended_without_evidence ||
        c->last_viewpoint_attempt_<0. || c->cells_.empty() ||
        (c->is_reachable_ && !c->vp_clusters_.empty() && !spent_stances && !revisitsLowerPass(body,{c->best_vp_}))) {
      if (trace) ROS_WARN_THROTTLE(3., "Survey completion waiting for label %d at %.2f %.2f %.2f: sampled %d, reachable %d, groups %zu, cells %zu",
          c->id_,c->center_.x(),c->center_.y(),c->center_.z(),int(c->last_viewpoint_attempt_>=0.),int(c->is_reachable_),c->vp_clusters_.size(),c->cells_.size());
      pending_survey_location_=c->center_;
      residual_sweep_.reset();
      return true;
    }
    if (!approachedClusterCells(*c,band)) {
      if (trace) ROS_WARN_THROTTLE(3., "Survey completion lacks measured face or approach for label %d at %.2f %.2f %.2f",
          c->id_,c->center_.x(),c->center_.y(),c->center_.z());
      pending_survey_location_=c->center_;
      residual_sweep_.reset();
      return true;
    }
    ++residual;
  }
  // The last wall label at a dead end can disappear even though a distant
  // branch is still visible from an earlier flight position. Complete its
  // recovery sweep before claiming the whole reachable map is exhausted.
  if (branch_anchors_.empty()) {residual_sweep_.reset();return false;}
  if (!residual_sweep_ || (body-residual_sweep_->first).norm()>radius)
    residual_sweep_=std::make_pair(body,next_branch_anchor_+branch_anchors_.size());
  if (next_branch_anchor_<residual_sweep_->second) {
    pending_survey_location_=branch_anchors_[next_branch_anchor_%branch_anchors_.size()];
    if (trace) ROS_WARN_THROTTLE(3., "Survey completion checking recovery anchors: %zu remaining",residual_sweep_->second-next_branch_anchor_);
    return true;
  }
  ROS_INFO("Reachable survey exhausted: %zu residual surface clusters without an executable observation after checking %zu recovery anchors",
                    residual,branch_anchors_.size());
  return false;
}

bool FrontierManager::hasInspectionFrontiers() const {
  // These clusters already passed the surface height/normal policy. Raw
  // frontier labels also contain excluded floor/ceiling cells.
  for(const auto &cluster:cluster_list_)
    if(cluster && !cluster->is_dormant_ && (!surface_continuity_ || !observedCluster(*cluster)))return true;
  return false;
}
