/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-03-01 15:10:53
 * @LastEditTime: 2024-03-01 17:18:07
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#include "pointcloud_topo/parallel_bubble_astar.h"
#include <array>

namespace {
// Non-overlapping clearance balls do not imply a collision between two grid
// vertices. Certify their segment by advancing inside the nearest-point ball;
// distance to a fixed obstacle set is 1-Lipschitz, so no fixed-step gap is left.
bool clearGridSegment(const LIOInterface::Ptr &map, const Eigen::Vector3f &a,
                      const Eigen::Vector3f &b, double clearance) {
  const double length=(b-a).norm();
  if(!std::isfinite(length))return false;
  if(length<1.e-7)return map->getDisToOcc(a)>clearance+1.e-4;
  const Eigen::Vector3f direction=(b-a)/length;
  double covered=0.;
  for(int count=0;count<4096;++count) {
    const Eigen::Vector3f point=a+float(covered)*direction;
    const double radius=map->getDisToOcc(point)-clearance-1.e-4;
    if(!std::isfinite(radius) || radius<=1.e-5)return false;
    if(radius>=length-covered)return true;
    covered+=radius;
  }
  return false;
}
}

void ParallelBubbleAstar::init(epic_ros::NodeHandle &nh, const LIOInterface::Ptr &lidar_map) {
  nh.param("bubble_astar/resolution_astar", resolution_, 0.2);
  nh.param("bubble_astar/lambda_heu", lambda_heu_, 100.0);
  nh.param("bubble_astar/allocate_num", allocate_num_, 1000000);
  nh.param("bubble_astar/safe_distance", safe_distance_, -1.0);
  nh.param("bubble_astar/debug", debug_, false);
  nh.param("flight_height/min_z", flight_min_z_, -1.e6);
  nh.param("flight_height/max_z", flight_max_z_, 1.e6);
  if(!std::isfinite(flight_min_z_) || !std::isfinite(flight_max_z_) || flight_min_z_>=flight_max_z_)
    throw std::invalid_argument("Invalid navigation height band");
  tie_breaker_ = 1.0 + 1.0 / 1000;
  this->lidar_map_interface_ = lidar_map;
  origin_ = lidar_map->lp_->global_map_min_boundary_;
  this->inv_resolution_ = 1.0 / resolution_;
  // frontier_manager_ = frontier_manager_;
}

bool ParallelBubbleAstar::isPositionSafe(const Eigen::Vector3f &position, const Eigen::Vector3f &bbox_min, const Eigen::Vector3f &bbox_max,
                                     unordered_set<Eigen::Vector3i, v3i_hash> &safe_set, unordered_set<Eigen::Vector3i, v3i_hash> &danger_set) {

  if(position.z()<flight_min_z_-1.e-5 || position.z()>flight_max_z_+1.e-5)return false;
  if (!lidar_map_interface_->IsInMap(position))
    return false;

  if (position.x() < bbox_min.x() || position.y() < bbox_min.y() || position.z() < bbox_min.z() ||
      position.x() > bbox_max.x() || position.y() > bbox_max.y() || position.z() > bbox_max.z())
    return false;
  Eigen::Vector3i idx;
  posToIndex(position, idx);
  if (safe_set.count(idx))
    return true;
  if (danger_set.count(idx))
    return false;

  double radius = lidar_map_interface_->getDisToOcc(position) - safe_distance_ - 1e-3;

  if (radius > 0) {
    Eigen::Vector3f cell_center;IndexToPos(cell_center,idx);
    const double offset=(cell_center-position).norm();
    // Cache only grid centers certified by this clearance sphere. The start
    // can be off-grid; caching its cell blindly can mark an unsafe center safe.
    if(radius>offset+1.e-4) {
      int width=int((radius-offset)*inv_resolution_/std::sqrt(3.))-1;
      width=std::clamp(width,0,3); // Bounded343-cell work, instead of min(0,width).
      for (int i = -width; i <= width; i++) {
      for (int j = -width; j <= width; j++) {
        for (int k = -width; k <= width; k++) {
          Eigen::Vector3i idx_tmp = idx + Eigen::Vector3i(i, j, k);
          safe_set.insert(idx_tmp);
        }
      }
    }
    }
    return true;
  }
  danger_set.insert(idx);
  return false;
}

int ParallelBubbleAstar::search(const Eigen::Vector3f &start, const Eigen::Vector3f &goal, vector<Eigen::Vector3f> &path, double timeout,
                                bool best_result, bool only_raycast, const Eigen::Vector3f &bbox_min, const Eigen::Vector3f &bbox_max) {
  path.clear();
  epic_ros::NodeHandle nh;double height_margin;
  nh.param("bubble_astar/endpoint_height_margin",height_margin,-1.);
  Eigen::Vector3f search_min=bbox_min,search_max=bbox_max;
  if(height_margin>=0.) {
    search_min.z()=std::max(double(search_min.z()),std::min(double(start.z()),double(goal.z()))-height_margin);
    search_max.z()=std::min(double(search_max.z()),std::max(double(start.z()),double(goal.z()))+height_margin);
  }
  if(!start.allFinite() || start.z()<flight_min_z_-1.e-5 || start.z()>flight_max_z_+1.e-5)return START_FAIL;
  if(!goal.allFinite() || goal.z()<flight_min_z_-1.e-5 || goal.z()>flight_max_z_+1.e-5)return END_FAIL;
  // step 1 one-shot
  vector<Eigen::Vector3f>().swap(path);
  double goal_r;
  goal_r = min(1.5, lidar_map_interface_->getDisToOcc(goal) - safe_distance_ - 1e-3);
  if (goal_r <= 1e-2) {
    return ParallelBubbleAstar::NO_PATH;
  }
  double len = (goal - start).norm();
  Eigen::Vector3f dir = (goal - start).normalized();
  Eigen::Vector3f curr_pos = start;
  vector<Eigen::Vector3f> path_tmp;
  path_tmp.push_back(curr_pos);
  while (true) {
    double step;
    step = min(1.5, lidar_map_interface_->getDisToOcc(curr_pos) - safe_distance_ - 1e-3);
    if (step <= 1e-2)
      break;
    if (step + goal_r > (goal - curr_pos).norm()) {
      path.swap(path_tmp);
      path.push_back((goal + curr_pos) / 2.0);
      path.push_back(goal);
      // ROS_INFO("one shot! reach goal");
      return ParallelBubbleAstar::REACH_END;
    } else {
      curr_pos += step * dir;
      path_tmp.push_back(curr_pos);
    }
    len -= step;
    if (len < 0) {
      ROS_ERROR("one shot has bug");
      exit(0);
    }
  }
  if (only_raycast) {
    return ParallelBubbleAstar::NO_PATH;
  }
  // ROS_INFO("one shot fail");

  auto getHeu = [&](const Eigen::Vector3f &x1, const Eigen::Vector3f &x2) -> double {
    if (best_result) {
      return tie_breaker_ * (x1 - x2).norm() * 0.9;
    }
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double dz = fabs(x1(2) - x2(2));
    return tie_breaker_ * (dx + dy + dz);
  };
  auto arriveGoal = [&](Eigen::Vector3f &curr_pos, double &curr_radius) -> bool {
    double curr_r = curr_radius;
    double dis2goal = (curr_pos - goal).norm();
    return (curr_r + goal_r > dis2goal);
  };

  std::priority_queue<Node::Ptr, std::vector<Node::Ptr>, NodeCompre> open_set;
  std::unordered_map<Eigen::Vector3i, Node::Ptr, v3i_hash> open_set_map;
  std::unordered_set<Eigen::Vector3i, v3i_hash> close_set, safe_set, danger_set;
  safe_set.reserve(1000);
  danger_set.reserve(1000);
  close_set.reserve(1000);
  open_set_map.reserve(1000);
  Node::Ptr curr_node = std::make_shared<Node>();
  curr_node->parent = nullptr;
  curr_node->position = start;
  if (!isPositionSafe(start, search_min, search_max, safe_set, danger_set)) {
    // ROS_ERROR("ParallelBubbleAstar: unsafe start point");
    return ParallelBubbleAstar::START_FAIL;
  }
  if (!isPositionSafe(goal, search_min, search_max, safe_set, danger_set)) {
    // ROS_ERROR("ParallelBubbleAstar: unsafe end point");
    return ParallelBubbleAstar::END_FAIL;
  }
  curr_node->g_score = 0.0;
  if (!best_result)
    curr_node->f_score = lambda_heu_ * getHeu(start, goal);
  else
    curr_node->f_score = getHeu(start, goal);

  open_set.push(curr_node);
  Eigen::Vector3i curr_idx;
  posToIndex(curr_node->position, curr_idx);
  open_set_map.insert(make_pair(curr_idx, curr_node));

  auto backtrack = [&](Node::Ptr curr_node) {
    while (curr_node->parent != nullptr) {
      path.push_back(curr_node->position);
      curr_node = curr_node->parent;
    }
    path.push_back(curr_node->position);
    std::reverse(path.begin(), path.end());
    path.push_back(goal);
  };

  epic_ros::Time t1 = epic_ros::Time::now();

  const float step_size=static_cast<float>(resolution_);
  const std::array<Eigen::Vector3f,6> steps{{
    {step_size, 0, 0}, {-step_size, 0, 0}, {0, step_size, 0},
    {0, -step_size, 0}, {0, 0, step_size}, {0, 0, -step_size}
  }};
  while (!open_set.empty()) {
    auto curr_node = open_set.top();
    double curr_radius = lidar_map_interface_->getDisToOcc(curr_node->position) - safe_distance_;
    if (arriveGoal(curr_node->position, curr_radius)) {
      backtrack(curr_node);
      bool safe = collisionCheck_shortenPath(path);
      if (!safe) {
        // ROS_ERROR("path just found not safe");
        return ParallelBubbleAstar::NO_PATH;
        // exit(1);
      }
      // ROS_INFO_STREAM("ParallelBubbleAstar: time cost = " << (epic_ros::Time::now() - t1).toSec() *
      // 1000 << "ms");
      return ParallelBubbleAstar::REACH_END;
    }
    if ((epic_ros::Time::now() - t1).toSec() > timeout) {
      // ROS_ERROR("ParallelBubbleAstar: TIME_OUT");
      return ParallelBubbleAstar::TIME_OUT;
    }
    open_set.pop();
    posToIndex(curr_node->position, curr_idx);
    open_set_map.erase(curr_idx);
    close_set.insert(curr_idx);
    Eigen::Vector3f nbr_pos;
    for (const auto &step : steps) {
      nbr_pos = curr_node->position + step;
      Eigen::Vector3i nbr_idx;
      posToIndex(nbr_pos, nbr_idx);
      IndexToPos(nbr_pos, nbr_idx);

      if (close_set.find(nbr_idx) != close_set.end())
        continue;
      if (!isPositionSafe(nbr_pos, search_min, search_max, safe_set, danger_set))
        continue;
      const double edge_length=(nbr_pos-curr_node->position).norm();
      if(curr_radius<edge_length) {
        const double neighbor_radius=lidar_map_interface_->getDisToOcc(nbr_pos)-safe_distance_;
        if(curr_radius+neighbor_radius<edge_length &&
           !clearGridSegment(lidar_map_interface_,curr_node->position,nbr_pos,safe_distance_))continue;
      }
      double tmp_g_score = step.norm() + curr_node->g_score;
      auto node_iter = open_set_map.find(nbr_idx);
      if (node_iter == open_set_map.end()) {
        auto nei_node = std::make_shared<Node>();
        nei_node->position = nbr_pos;
        nei_node->parent = curr_node;
        nei_node->g_score = tmp_g_score;
        if (!best_result)
          nei_node->f_score = lambda_heu_ * getHeu(nbr_pos, goal) + tmp_g_score;
        else
          nei_node->f_score = getHeu(nbr_pos, goal) + tmp_g_score;

        open_set_map.insert(make_pair(nbr_idx, nei_node));
        open_set.push(nei_node);
      }
    }
  }
  // ROS_INFO("all space searched");
  return ParallelBubbleAstar::NO_PATH;
}

void ParallelBubbleAstar::reset() {
  unordered_set<Eigen::Vector3i, v3i_hash>().swap(safe_node);
  unordered_set<Eigen::Vector3i, v3i_hash>().swap(dangerous_node);
}

void ParallelBubbleAstar::posToIndex(const Eigen::Vector3f &pt, Eigen::Vector3i &idx) {
  idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
}

void ParallelBubbleAstar::IndexToPos(Eigen::Vector3f &pt, Eigen::Vector3i &idx) {
  pt = ((idx.cast<float>() + Eigen::Vector3f(0.5, 0.5, 0.5)) * resolution_ + origin_);
}

bool ParallelBubbleAstar::collisionCheck_shortenPath(vector<Eigen::Vector3f> &path, double clearance) {
  // if (path.size() < 2)
  //   return false;
  // vector<Eigen::Vector3f> path_shorten;
  // std::vector<double> raduis_lis;
  // for (int i = 0; i < path.size(); i++) {
  //   double dis = lidar_map_interface_->getDisToOcc(path[i]) - safe_distance_;
  //   raduis_lis.push_back(dis);
  //   if (raduis_lis.back() < 1e-3) {
  //     return false;
  //   }
  // }
  // vector<int> path_shorten_idx;
  // path_shorten_idx.push_back(0);
  // for (int i = 1; i < path.size() - 1; i++) {
  //   Eigen::Vector3f back_node = path[path_shorten_idx.back()];
  //   Eigen::Vector3f cur = path[i];
  //   Eigen::Vector3f next = path[i + 1];
  //   bool cur_connect = (cur - back_node).norm() < raduis_lis[i] + raduis_lis[path_shorten_idx.back()];
  //   bool next_connect = ((next - back_node).norm() < raduis_lis[i + 1] + raduis_lis[path_shorten_idx.back()]);
  //   if (cur_connect && next_connect)
  //     continue;
  //   else if (cur_connect && !next_connect) {
  //     path_shorten_idx.push_back(i);
  //   } else if (!cur_connect) {
  //     return false;
  //   }
  // }
  // path_shorten_idx.push_back(path.size() - 1);

  // for (int i = 0; i < path_shorten_idx.size() ; i++) {
  //   path_shorten.push_back(path[path_shorten_idx[i]]);
  // }
  // path.swap(path_shorten);
  // return true;
  if(clearance<0.)clearance=safe_distance_;
  if(!std::isfinite(clearance) || clearance<0.)return false;
  if (path.size() < 2)
    return false;
  std::vector<Eigen::Vector3f> path_shorten;
  std::vector<double> raduis_lis;
  raduis_lis.reserve(path.size());
  for (int i = 0; i < path.size(); i++) {
    if(!path[i].allFinite())return false;
    double dis = min(2.0, lidar_map_interface_->getDisToOcc(path[i]) - clearance);
    raduis_lis.push_back(dis);
    if (raduis_lis.back() < 1e-3) {
      return false;
    }
  }

  std::deque<int> indices;
  indices.push_front(path.size() - 1); // goal
  for (int i = path.size() - 1; i > 0; i--) {
    double i_raduis = raduis_lis[i];
    bool connected = false;
    for (int j = 0; j < i; j++) {
      double j_raduis = raduis_lis[j];
      double distance = (path[i] - path[j]).norm();
      if (i_raduis + j_raduis + 1e-3 > distance) {
        indices.push_front(j);
        connected = true;
        i = j + 1;
        break;
      }
    }
    if (!connected) {
      // Retain a certified original grid edge if no sphere-overlap shortcut
      // exists. This preserves the configured clearance in narrow passages.
      if(!clearGridSegment(lidar_map_interface_,path[i-1],path[i],clearance))return false;
      indices.push_front(i-1);
    }
  }
  for (auto &idx : indices) {
    path_shorten.emplace_back(path[idx]);
  }
  path.swap(path_shorten);
  return true;
}

void ParallelBubbleAstar::calculatePathCost(const vector<Eigen::Vector3f> &path, double &cost) {
  cost = 0;
  for (int i = 0; i < path.size() - 1; i++) {
    cost += (path[i + 1] - path[i]).norm();
  }
}
