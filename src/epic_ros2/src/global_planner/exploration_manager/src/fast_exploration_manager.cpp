#include <epic_planner/frontier_route.h>
/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-02-25 15:00:51
 * @LastEditTime: 2024-03-12 22:15:11
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#include <boost/lexical_cast.hpp>
#include <epic_planner/expl_data.h>
#include <epic_planner/fast_exploration_manager.h>
#include <fstream>
#include <iostream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_manage/planner_manager.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <visualization_msgs/msg/marker.hpp>
using namespace std;
using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {}

FastExplorationManager::~FastExplorationManager() {}

void FastExplorationManager::initialize(
    epic_ros::NodeHandle &nh, FrontierManager::Ptr frt_manager,
    FastPlannerManager::Ptr planner_manager) {

  frontier_manager_ptr_ = frt_manager;
  planner_manager_ = planner_manager;

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);
  ed_->next_goal_node_ = make_shared<TopoNode>();
  ed_->next_goal_node_->center_.setZero();
  ed_->next_goal_node_->yaw_ = 0.f;

  ep_->a_avg_ = tan(planner_manager_->gcopter_config_->maxTiltAngle) *
                planner_manager_->gcopter_config_->gravAcc;
  ep_->v_max_ = planner_manager_->gcopter_config_->maxVelMag;
  ep_->yaw_v_max_ = planner_manager_->gcopter_config_->yaw_max_vel;
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.getParam("viewpoint_param/global_viewpoint_num",
              ep_->global_viewpoint_num_);
  nh.getParam("view_graph", ep_->view_graph_);
  nh.getParam("viewpoint_param/local_viewpoint_num", ep_->local_viewpoint_num_);
  nh.getParam("global_planning/w_vdir", ep_->w_vdir_);
  nh.param("global_planning/use_velocity_direction_cost",use_velocity_direction_cost_,false);
  nh.param("global_planning/directional_frontiers",directional_frontiers_,false);
  if(use_velocity_direction_cost_)ROS_INFO("Global first-leg direction cost enabled: w=%.2f, native LKH retained",ep_->w_vdir_);
  nh.getParam("global_planning/w_yawdir", ep_->w_yawdir_);
  Eigen::Vector3d origin, size;
  ofstream par_file(ep_->tsp_dir_ + "/single.par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "MOVE_TYPE = 2\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 10\n";
  epic_ros::Duration(1.0).sleep();
}

void FastExplorationManager::goalCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
  // 提取四元数
  double roll, pitch;
  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.orientation, quat);

  // 将四元数转换为Euler角
  tf2::Matrix3x3(quat).getRPY(roll, pitch, goal_yaw);
}

double FastExplorationManager::initialDirectionCost(
    const vector<Eigen::Vector3f> &path,const Eigen::Vector3d &velocity,
    double weight,double yaw_rate) {
  if(path.size()<2 || !velocity.allFinite() || velocity.head<2>().norm()<=.5 ||
      !std::isfinite(weight) || weight<=0. || !std::isfinite(yaw_rate) || yaw_rate<=0.)return 0.;
  // Use the first2m of the certified native graph route, not the direct
  // goal bearing which can point through a shelf on an otherwise forward route.
  Eigen::Vector3f point=path.front();double remaining=2.;
  if(!point.allFinite())return 0.;
  for(size_t i=1;i<path.size();++i) {
    if(!path[i].allFinite())return 0.;
    const Eigen::Vector3f segment=path[i]-point;const double length=segment.norm();
    if(length<1.e-6)continue;
    if(length>=remaining){point+=segment*float(remaining/length);break;}
    point=path[i];remaining-=length;
  }
  const Eigen::Vector2d direction=(point-path.front()).head<2>().cast<double>();
  if(direction.norm()<.1)return 0.;
  const double angle=std::abs(std::remainder(atan2(direction.y(),direction.x())-
                     atan2(velocity.y(),velocity.x()),2.*M_PI));
  return weight*angle/yaw_rate;
}

double FastExplorationManager::getPathCost(TopoNode::Ptr &n1,
                                           Eigen::Vector3d v1, float &yaw1,
                                           TopoNode::Ptr &n2, float &yaw2,
                                           double *initial_turn_cost) {
  if(initial_turn_cost)*initial_turn_cost=0.;
  auto estimateCost = [&](TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1,
                          TopoNode::Ptr &n2, float &yaw2, int res,
                          vector<Eigen::Vector3f> &path) -> double {
    double len_cost, yaw_cost, dir_cost;
    len_cost = yaw_cost = dir_cost = 0.0;
    if (res == BubbleAstar::NO_PATH)
      return 2e3 + (n1->center_ - n2->center_)
                       .norm(); // 使用一个大的时间值表示无法到达
    if (res == BubbleAstar::START_FAIL || res == BubbleAstar::END_FAIL)
      return 2e3 +
             (n1->center_ - n2->center_).norm(); // 同上，用于不同的错误情况

    len_cost = 0.0;
    for (int i = 0; i < path.size() - 1; ++i)
      len_cost += ((path[i + 1] - path[i]).norm() +
                   0.5 * fabs(path[i + 1].z() - path[i].z()));
    len_cost /= (ep_->v_max_ / 2.0);

    // if (v1.norm() > 1e-3) {
    //   Eigen::Vector3f dir = n2->center_ - n1->center_;
    //   dir.normalize();
    //   Eigen::Vector3f v_dir = v1.normalized().cast<float>();
    //   float yaw1 = atan2(dir.y(), dir.x());
    //   float yaw2 = atan2(v_dir.y(), v_dir.x());
    //   float diff = yaw1 - yaw2;
    //   while (diff > M_PI)
    //     diff -= 2.0 * M_PI;
    //   while (diff < -M_PI)
    //     diff += 2.0 * M_PI;
    //   dir_cost = ep_->w_vdir_ * (fabs(diff) /
    //   planner_manager_->gcopter_config_->yaw_max_vel);
    // }

    // if (path.size() >= 2) {
    //   planner_manager_->calculateTimelb(path, yaw1, yaw2, yaw_cost);
    //   yaw_cost *= ep_->w_yawdir_;
    // }

    return len_cost + dir_cost;
    // return len_cost + dir_cost;
  };
  vector<Eigen::Vector3f> path;
  int res = planner_manager_->fast_searcher_->topoSearch(n1, n2, 1e-2, path);
  const double cost=estimateCost(n1, v1, yaw1, n2, yaw2, res, path);
  if(initial_turn_cost && cost<2e3)
    *initial_turn_cost=directional_frontiers_ ? frontierContinuationCost(path,v1.cast<float>()) :
        initialDirectionCost(path,v1,ep_->w_vdir_,planner_manager_->gcopter_config_->yaw_max_vel);
  return cost;
}

double FastExplorationManager::getPathCostWithoutTopo(TopoNode::Ptr &n1,
                                                      Eigen::Vector3d v1,
                                                      float &yaw1,
                                                      TopoNode::Ptr &n2,
                                                      float &yaw2) {
  vector<Eigen::Vector3f> path;
  int res = planner_manager_->parallel_path_finder_->search(
      n1->center_, n2->center_, path, 1.0, false);
  if (res != ParallelBubbleAstar::REACH_END)
    return 2e3;
  double cost;
  planner_manager_->parallel_path_finder_->calculatePathCost(path, cost);
  return cost;
}

int FastExplorationManager::frontierStatus() {
  if(frontier_manager_ptr_->hasInspectionFrontiers()) {
    frontier_empty_since_=-1.;return FRONTIER_PENDING;
  }
  const double now=epic_ros::Time::now().toSec();
  if(frontier_empty_since_<0.)frontier_empty_since_=now;
  epic_ros::NodeHandle nh;double confirm;
  nh.param("exploration/frontier_empty_confirmation",confirm,0.);
  return now-frontier_empty_since_<confirm ? FRONTIER_PENDING : NO_FRONTIER;
}

int FastExplorationManager::planGlobalPath(const Eigen::Vector3d &pos,
                                           const Eigen::Vector3d &vel) {
  bool bm_without_topo = false;
  auto estimiateVdirCost = [&](const TopoNode::Ptr &n1,
                               const Eigen::Vector3d &v1,
                               const TopoNode::Ptr &n2) -> double {
    Eigen::Vector3f dir = n2->center_ - n1->center_;
    dir.normalize();
    Eigen::Vector3f v_dir = v1.normalized().cast<float>();
    float yaw1 = atan2(dir.y(), dir.x());
    float yaw2 = atan2(v_dir.y(), v_dir.x());
    float diff = yaw1 - yaw2;
    while (diff > M_PI)
      diff -= 2.0 * M_PI;
    while (diff < -M_PI)
      diff += 2.0 * M_PI;
    return ep_->w_vdir_ *
           (fabs(diff) / planner_manager_->gcopter_config_->yaw_max_vel);
  };
  epic_ros::Time start = epic_ros::Time::now();
  vector<TopoNode::Ptr> viewpoints;
  // Visitation/handoff uses the latest measured body pose. The graph odom
  // anchor follows scan updates and can lag behind a fast moving vehicle.
  // Keep that graph anchor intact for certified native routing.
  Eigen::Vector3f current_body_position=pos.cast<float>();
  Eigen::Vector3d selection_velocity=vel;
  if(directional_frontiers_) {
    if(vel.head<2>().norm()>.75)preferred_heading_=vel.cast<float>().normalized();
    selection_velocity=preferred_heading_.cast<double>();
  }
  frontier_manager_ptr_->generateTSPViewpoints(current_body_position,viewpoints);

  if (viewpoints.empty()) {
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    ed_->global_tour_.clear();
    return frontierStatus();
  }

  frontier_empty_since_=-1.;
  epic_ros::Time t1 = epic_ros::Time::now();
  planner_manager_->topo_graph_->insertNodes(viewpoints, false);
  updateGoalNode();
  epic_ros::Time t2 = epic_ros::Time::now();
  cout << "insert viewpoint to graph time: " << (t2 - t1).toSec() * 1000
       << " ms" << endl;
  float curr_yaw = (float)planner_manager_->local_data_.curr_yaw_;
  vector<double> distance_odom2vp(viewpoints.size(), 0);
  vector<double> initial_turn_cost(viewpoints.size(),0.);
  vector<double> distance_lastgoal2vp(viewpoints.size(), 0);
  double dis2last_goal = 5e3;
  if (planner_manager_->lidar_map_interface_->getDisToOcc(
          ed_->next_goal_node_->center_) >
      planner_manager_->parallel_path_finder_->safe_distance_ + 0.1) {
    dis2last_goal = getPathCost(planner_manager_->topo_graph_->odom_node_,
                                Eigen::Vector3d::Zero(), curr_yaw,
                                ed_->next_goal_node_, curr_yaw);
  }
  static double last_frame_value = dis2last_goal;
  bool last_goal_reachable = dis2last_goal < 2e3 && !external_reselect_from_current_;
  external_reselect_from_current_=false;
  // last_goal_reachable = false;

  if (last_goal_reachable && (dis2last_goal < 1.5 * last_frame_value)) {
    last_frame_value = dis2last_goal;
  } else {
    last_goal_reachable = false;
  }

  epic_ros::Time t_start_cvp_1 = epic_ros::Time::now();
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 0; i < viewpoints.size(); ++i) {
    if (last_goal_reachable) {
      distance_lastgoal2vp[i] =
          getPathCost(ed_->next_goal_node_, Eigen::Vector3d::Zero(),
                      viewpoints[i]->yaw_, viewpoints[i], viewpoints[i]->yaw_);
      distance_odom2vp[i] =
          getPathCost(planner_manager_->topo_graph_->odom_node_, selection_velocity, curr_yaw,
                      viewpoints[i], viewpoints[i]->yaw_,
                      use_velocity_direction_cost_ ? &initial_turn_cost[i] : nullptr);

    } else {
      distance_lastgoal2vp[i] =
          getPathCost(planner_manager_->topo_graph_->odom_node_, selection_velocity, curr_yaw,
                      viewpoints[i], viewpoints[i]->yaw_,
                      use_velocity_direction_cost_ ? &initial_turn_cost[i] : nullptr);
      distance_odom2vp[i] = distance_lastgoal2vp[i];
    }
  }
  epic_ros::Time t_end_cvp_1 = epic_ros::Time::now();
  if (bm_without_topo) {
    omp_set_num_threads(4);
    // clang-format off
    #pragma omp parallel for
    // clang-format on
    for (int i = 0; i < viewpoints.size(); ++i) {
      if (last_goal_reachable) {
        distance_lastgoal2vp[i] = getPathCostWithoutTopo(
            ed_->next_goal_node_, Eigen::Vector3d::Zero(), viewpoints[i]->yaw_,
            viewpoints[i], viewpoints[i]->yaw_);
        distance_odom2vp[i] = getPathCostWithoutTopo(
            planner_manager_->topo_graph_->odom_node_, selection_velocity, curr_yaw,
            viewpoints[i], viewpoints[i]->yaw_);

      } else {
        distance_lastgoal2vp[i] = getPathCostWithoutTopo(
            planner_manager_->topo_graph_->odom_node_, selection_velocity, curr_yaw,
            viewpoints[i], viewpoints[i]->yaw_);
        distance_odom2vp[i] = distance_lastgoal2vp[i];
      }
    }
    epic_ros::Time t_end_cvp_2 = epic_ros::Time::now();
    double cost_mat_with_topo = (t_end_cvp_1 - t_start_cvp_1).toSec() * 1000;
    double cost_mat_without_topo = (t_end_cvp_2 - t_end_cvp_1).toSec() * 1000;
    cout << "cost mat topo: " << cost_mat_with_topo << "ms" << endl;
    cout << "cost mat point cloud: " << cost_mat_without_topo << "ms" << endl;
  }

  vector<TopoNode::Ptr> viewpoint_reachable;
  vector<double> viewpoint_reachable_distance, viewpoint_reachable_distance2;
  vector<double> viewpoint_reachable_turn_cost;
  for (int i = 0; i < distance_lastgoal2vp.size(); ++i) {
    if (distance_odom2vp[i] > 2e3)
      continue;
    if (last_goal_reachable) {
      viewpoint_reachable_distance.emplace_back(distance_lastgoal2vp[i]);

    } else {
      viewpoint_reachable_distance.emplace_back(distance_odom2vp[i]);
    }
    viewpoint_reachable_distance2.emplace_back(distance_odom2vp[i]);
    viewpoint_reachable_turn_cost.push_back(initial_turn_cost[i]);
    viewpoint_reachable.emplace_back(viewpoints[i]);
  }

  // A sampled viewpoint can belong to a disconnected topology component even
  // though free space connects it to the robot. Repair only the nearest native
  // candidate, once per global update, before deciding to wait for map growth.
  if (viewpoint_reachable.empty() && !viewpoints.empty()) {
    auto graph=planner_manager_->topo_graph_;
    auto target=viewpoints.front();
    if(directional_frontiers_)target=*std::min_element(viewpoints.begin(),viewpoints.end(),
        [&](const TopoNode::Ptr&a,const TopoNode::Ptr&b) {
          auto priority=[&](const TopoNode::Ptr&v) {
            return frontierContinuationCost({graph->odom_node_->center_,v->center_},preferred_heading_)+
                (v->center_-graph->odom_node_->center_).norm();
          };
          return priority(a)<priority(b);
        });
    vector<Eigen::Vector3f> connector;
    auto astar=graph->parallel_bubble_astar_;
    if(astar->search(graph->odom_node_->center_,target->center_,connector,.08,false)==ParallelBubbleAstar::REACH_END &&
        connector.size()>=2 && astar->collisionCheck_shortenPath(connector)) {
      double length;astar->calculatePathCost(connector,length);
      graph->odom_node_->neighbors_.insert(target);target->neighbors_.insert(graph->odom_node_);
      graph->odom_node_->paths_[target]=connector;
      target->paths_[graph->odom_node_]=vector<Eigen::Vector3f>(connector.rbegin(),connector.rend());
      graph->odom_node_->weight_[target]=target->weight_[graph->odom_node_]=length;
      viewpoint_reachable.push_back(target);
      viewpoint_reachable_distance.push_back(length/(ep_->v_max_/2.));
      viewpoint_reachable_distance2.push_back(viewpoint_reachable_distance.back());
      viewpoint_reachable_turn_cost.push_back(directional_frontiers_ ?
          frontierContinuationCost(connector,preferred_heading_) : 0.);
      ROS_INFO("Reconnected native frontier viewpoint through collision-checked free space");
    }
  }
  if (viewpoint_reachable.empty()) {
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    ed_->global_tour_.clear();
    return frontierStatus();
  }

  if (viewpoint_reachable.size() == 1) {
    ed_->global_tour_.clear();
    ed_->global_tour_.emplace_back(pos.cast<float>());
    ed_->global_tour_.emplace_back(viewpoint_reachable.front()->center_);
    planner_manager_->local_data_.end_yaw_ = viewpoint_reachable.front()->yaw_;
    updateGoalNode(viewpoint_reachable.front());
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    planner_manager_->graph_visualizer_->vizTour(ed_->global_tour_, VizColor::RED, "global");
    return SUCCEED;
  }

  int dim = viewpoint_reachable.size() + 1;
  Eigen::MatrixXd mat;
  mat.resize(dim, dim);
  mat.setZero();
  for (int i = 1; i < dim; ++i) {
    mat(0, i) = viewpoint_reachable_distance[i - 1];
  }

  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 1; i < dim; i++) {
    for (int j = i + 1; j < dim; j++) {
      mat(i, j) = mat(j, i) = getPathCost(
          viewpoint_reachable[i - 1], Eigen::Vector3d(0, 0, 0),
          viewpoint_reachable[i - 1]->yaw_, viewpoint_reachable[j - 1],
          viewpoint_reachable[j - 1]->yaw_);
    }
  }
  // trick 往远走
  for (int i = 1; i < dim; ++i) {
    mat(i, 0) = 2e3 - viewpoint_reachable_distance2[i - 1] * 0.2;
  }
  for (int i = 0; i < dim; ++i) {
    for (int j = 1; j < dim; ++j) {
      for (int k = 1; k < dim; ++k) {
        if (mat(i, j) > mat(i, k) + mat(k, j)) {
          mat(i, j) = mat(i, k) + mat(k, j) + 1e-2;
        }
      }
    }
  }
  if(use_velocity_direction_cost_)for(int i=1;i<dim;++i)
    mat(0,i)+=viewpoint_reachable_turn_cost[i-1];
  vector<int> indices;
  indices.reserve(dim);
  epic_ros::Time start_tsp = epic_ros::Time::now();
  cout << "calculate tsp cost matrix cost " << (start_tsp - t2).toSec() * 1000
       << "ms" << endl;
  solveLHK(mat, indices);
  epic_ros::Time end_tsp = epic_ros::Time::now();
  cout << "lkh solver cost: " << (end_tsp - start_tsp).toSec() * 1000 << "ms"
       << endl;
  // if ((end_tsp - start_tsp).toSec() * 1000 > 100)
  //   exit(0);
  ed_->global_tour_.clear();
  for (auto &i : indices) {
    if (i == 0) {
      ed_->global_tour_.push_back(
          planner_manager_->topo_graph_->odom_node_->center_);
    } else {
      ed_->global_tour_.emplace_back(viewpoint_reachable[i - 1]->center_);
    }
  }
  if (!last_goal_reachable)
    last_frame_value = viewpoint_reachable_distance[indices[1] - 1];

  epic_ros::Time end = epic_ros::Time::now();
  // Transfer the selected viewpoint's certified native connections before its
  // temporary node is removed. A bounded reconnect may not find the same edge.
  const auto selected_viewpoint=viewpoint_reachable[indices[1]-1];
  updateGoalNode(selected_viewpoint);
  planner_manager_->topo_graph_->removeNodes(viewpoints);
  planner_manager_->graph_visualizer_->vizTour(ed_->global_tour_, VizColor::RED,
                                               "global");

  planner_manager_->local_data_.end_yaw_ =
      viewpoint_reachable[indices[1] - 1]->yaw_;
  return SUCCEED;
}

void FastExplorationManager::solveLHK(Eigen::MatrixXd &cost_mat,
                                      vector<int> &indices) {
  // Solve linear homogeneous kernel
  // Write params and cost matrix to problem file
  int dimension = cost_mat.rows();
  if (dimension < 3)
    return;
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  // Problem specification part, follow the format of TSPLIB

  string prob_spec =
      "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " +
  // to_string(dimension) +
  //     "\nEDGE_WEIGHT_TYPE : "
  //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;

  // Use Asymmetric TSP
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {

    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == -1)
      break;
    indices.push_back(id - 1); // Idx of solver-2 == Idx of frontier
  }

  res_file.close();
}

bool FastExplorationManager::reanchorLocalPath(vector<Eigen::Vector3f> &path,
    const Eigen::Vector3f &boundary,const Eigen::Vector3f &velocity) {
  if(path.empty() || !boundary.allFinite() || !velocity.allFinite())return false;
  auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
  auto lio=planner_manager_->lidar_map_interface_;
  const auto goal=path.back();
  epic_ros::NodeHandle height_nh;double height_margin;
  height_nh.param("bubble_astar/endpoint_height_margin",height_margin,-1.);
  if(height_margin>=0.) {
    const double lower=std::min(boundary.z(),goal.z())-height_margin;
    const double upper=std::max(boundary.z(),goal.z())+height_margin;
    if(std::any_of(path.begin(),path.end(),[&](const Eigen::Vector3f&p){return p.z()<lower || p.z()>upper;})) {
      vector<Eigen::Vector3f> repaired;
      if(astar->search(boundary,goal,repaired,.08,false)!=ParallelBubbleAstar::REACH_END) {
        ROS_WARN("Reject cached guide outside endpoint height band; waiting for native bounded reconnect");
        path.clear();return false;
      }
      path.swap(repaired);
      ROS_INFO("Repaired native cached guide within measured endpoint height band");
    }
  }
  if(!goal.allFinite() || goal.z()<astar->flight_min_z_-1.e-5 ||
      goal.z()>astar->flight_max_z_+1.e-5 ||
      lio->getDisToOcc(goal)<=astar->safe_distance_+.011) {
    path.clear();return false;
  }
  auto attach=[&](size_t join,vector<Eigen::Vector3f> connector) {
    connector.insert(connector.end(),path.begin()+join+1,path.end());
    path.swap(connector);
  };
  Eigen::Vector3f direction=velocity;
  if(direction.norm()<.1f)direction=goal-boundary;
  if(direction.norm()>.1f) {
    direction.normalize();
    // Prefer the latest safe point near the moving boundary, discarding OLD
    // departure loops rather than picking their earliest nearby occurrence.
    for(size_t i=path.size();i-->0;) {
      const Eigen::Vector3f delta=path[i]-boundary;
      if(delta.norm()>3.f || delta.dot(direction)<.05f)continue;
      vector<Eigen::Vector3f> connector;
      if(astar->search(boundary,path[i],connector,.002,false,true)==ParallelBubbleAstar::REACH_END) {
        attach(i,std::move(connector));return true;
      }
    }
  }
  // Repair only the departure into the existing native graph guide. Rebuilding
  // the distant global goal here turns a local reconnect into a warehouse-wide
  // grid search. The suffix is guidance; complete candidate checks still apply.
  int attempts=0;
  for(size_t i=path.size();i-->0 && attempts<2;) {
    const float distance=(path[i]-boundary).norm();
    if(distance<.05f || distance>3.f ||
        lio->getDisToOcc(path[i])<=astar->safe_distance_+.011)continue;
    ++attempts;vector<Eigen::Vector3f> connector;
    if(astar->search(boundary,path[i],connector,.03,false)==ParallelBubbleAstar::REACH_END) {
      attach(i,std::move(connector));
      ROS_INFO("Native A* repaired local departure; cached global guide/goal retained");return true;
    }
  }
  const auto &config=planner_manager_->gcopter_config_;
  if(config && config->startCorridorRecovery) {
    const float margin=config->dilateRadiusHard+config->startRecoveryMargin;
    vector<size_t> candidates;
    for(size_t i=0;i<path.size();++i)
      if((path[i]-boundary).norm()<=3.f &&
          lio->getDisToOcc(path[i])>astar->safe_distance_+.011)candidates.push_back(i);
    std::stable_sort(candidates.begin(),candidates.end(),[&](size_t a,size_t b) {
      const float da=(path[a]-boundary).squaredNorm(),db=(path[b]-boundary).squaredNorm();
      return da==db ? a>b : da<db;
    });
    for(size_t i=0;i<std::min(size_t(8),candidates.size());++i) {
      const size_t join=candidates[i];const auto p=path[join];
      Eigen::Vector3f current=boundary;bool certified=false;
      for(int step=0;step<64;++step) {
        const float radius=lio->getDisToOcc(current)-margin;
        const Eigen::Vector3f delta=p-current;const float distance=delta.norm();
        if(!std::isfinite(radius) || radius<=1.e-4f)break;
        if(distance<=radius){certified=true;break;}
        current+=delta*(.9f*radius/distance);
      }
      if(!certified)continue;
      attach(join,{boundary,p});
      ROS_INFO("Local departure recovered with certified %.3fm clearance; cached native guide retained",margin);
      return true;
    }
  }
  // Sparse cached guides may have no nearby point. Preserve the bounded full
  // native fallback, never prepend an unchecked leg to an old departure.
  vector<Eigen::Vector3f> rebuilt;
  const int status=astar->search(boundary,goal,rebuilt,.08,false);
  if(status!=ParallelBubbleAstar::REACH_END || rebuilt.size()<2 ||
      (rebuilt.back()-goal).norm()>.2f) {
    ROS_WARN("Cannot safely reanchor cached local guide; native A* status %d, start=(%.2f %.2f %.2f) clearance=%.3f, goal=(%.2f %.2f %.2f) clearance=%.3f",
        status,boundary.x(),boundary.y(),boundary.z(),lio->getDisToOcc(boundary),
        goal.x(),goal.y(),goal.z(),lio->getDisToOcc(goal));
    path.clear();return false;
  }
  path.swap(rebuilt);return true;
}

bool FastExplorationManager::extendTourPath(vector<Eigen::Vector3f> &path) {
  epic_ros::NodeHandle nh;double limit;
  nh.param("exploration/tour_lookahead_distance",limit,0.);
  double corner_speed;nh.param("exploration/corridor_corner_preview_speed",corner_speed,0.);
  if(corner_speed>0. && planner_manager_->local_data_.curr_vel_.norm()>corner_speed)return false;
  if(!std::isfinite(limit) || limit<=0. || limit>40. || path.size()<2 ||
      ed_->global_tour_.size()<3 || (path.back()-ed_->global_tour_[1]).norm()>.1f)return false;
  double used=0.;
  for(size_t i=1;i<path.size();++i) {
    if(!path[i].allFinite() || !path[i-1].allFinite())return false;
    used+=(path[i]-path[i-1]).norm();
  }
  if(used>=limit-.1)return false;
  auto extended=path;bool changed=false;
  for(size_t j=2;j<ed_->global_tour_.size() && used<limit-.1;++j) {
    vector<Eigen::Vector3f> segment;
    if(planner_manager_->topo_graph_->parallel_bubble_astar_->search(
         extended.back(),ed_->global_tour_[j],segment,.01,false)!=ParallelBubbleAstar::REACH_END || segment.size()<2)break;
    if(!planner_manager_->topo_graph_->parallel_bubble_astar_->collisionCheck_shortenPath(segment))continue;
    // Defer a native tour leg which immediately reverses the current pass.
    // Its frontier remains unvisited and may be selected at the next junction.
    // Test the actual free-space route, not a bearing through a shelf.
    if(directional_frontiers_ && extended.size()>1 &&
        frontierContinuationCost(segment,extended.back()-extended[extended.size()-2])>=300.)continue;
    auto prefix=FastPlannerManager::boundedGuidePrefix(segment,limit-used);
    if(prefix.size()<2)break;
    for(size_t i=1;i<prefix.size();++i) {
      used+=(prefix[i]-prefix[i-1]).norm();
      extended.push_back(prefix[i].cast<float>());
    }
    changed=true;
    if((extended.back()-ed_->global_tour_[j]).norm()>.05f)break;
  }
  if(!changed)return false;
  path.swap(extended);
  ROS_INFO("Native TSP corner preview: %.1fm guide; original goal/order and trajectory safety checks retained",used);
  return true;
}

void FastExplorationManager::updateGoalNode(const TopoNode::Ptr &validated_source) {
  if (ed_->global_tour_.size() < 2)
    return;
  Eigen::Vector3f goal = ed_->global_tour_[1];
  vector<pair<TopoNode::Ptr,vector<Eigen::Vector3f>>> preserved;
  unordered_set<TopoNode::Ptr> preserved_anchors,persistent;
  vector<TopoNode::Ptr> queue;
  const auto odom_node=planner_manager_->topo_graph_->odom_node_;
  if(odom_node){persistent.insert(odom_node);queue.push_back(odom_node);}
  for(size_t i=0;i<queue.size();++i)for(const auto&next:queue[i]->neighbors_)
    if(next && !next->is_viewpoint_ && persistent.insert(next).second)queue.push_back(next);
  auto source=validated_source;
  if(!source && (ed_->next_goal_node_->center_-goal).norm()<1.e-4f)
    source=ed_->next_goal_node_;
  auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
  auto preserve=[&](const TopoNode::Ptr &anchor,vector<Eigen::Vector3f> path) {
    if(!anchor || anchor==ed_->next_goal_node_ || !persistent.count(anchor) || path.size()<2 ||
        (path.front()-goal).norm()>.2f || (path.back()-anchor->center_).norm()>.2f)return;
    path.front()=goal;path.back()=anchor->center_;
    for(const auto&p:path)if(!p.allFinite() || p.z()<astar->flight_min_z_-1.e-5 ||
        p.z()>astar->flight_max_z_+1.e-5)return;
    if(astar->collisionCheck_shortenPath(path) && preserved_anchors.insert(anchor).second)
      preserved.emplace_back(anchor,std::move(path));
  };
  if(source && (source->center_-goal).norm()<1.e-4f) {
    for(const auto&anchor:source->neighbors_) {
      if(!anchor || anchor->is_viewpoint_)continue;
      const auto found=source->paths_.find(anchor);
      if(found!=source->paths_.end())preserve(anchor,found->second);
    }
    if(preserved.empty() && odom_node) {
      // A selected target may connect through other temporary viewpoints.
      // Materialize that ORIGINAL graph route back to its persistent prefix,
      // so deleting the temporary nodes cannot delete the selected route.
      vector<TopoNode::Ptr> chain;
      if(planner_manager_->topo_graph_->graphSearch(odom_node,source,chain,.01) && chain.size()>1) {
        size_t anchor_index=0;
        while(anchor_index+1<chain.size() && persistent.count(chain[anchor_index+1]))++anchor_index;
        vector<Eigen::Vector3f> route;bool valid=anchor_index+1<chain.size();
        for(size_t i=anchor_index;valid && i+1<chain.size();++i) {
          const auto edge=chain[i]->paths_.find(chain[i+1]);
          if(edge==chain[i]->paths_.end() || edge->second.size()<2 ||
              (edge->second.front()-chain[i]->center_).norm()>.2f ||
              (edge->second.back()-chain[i+1]->center_).norm()>.2f){valid=false;break;}
          auto part=edge->second;part.front()=chain[i]->center_;part.back()=chain[i+1]->center_;
          route.insert(route.end(),part.begin()+(route.empty()?0:1),part.end());
        }
        if(valid){std::reverse(route.begin(),route.end());preserve(chain[anchor_index],std::move(route));}
      }
    }
  }
  double goal_connection_timeout;
  epic_ros::NodeHandle nh;
  nh.param("parallel_astar/goal_connection_timeout", goal_connection_timeout, .001);
  if (!std::isfinite(goal_connection_timeout) || goal_connection_timeout <= 0.)
    throw std::invalid_argument("Goal connection timeout must be positive");


  struct PairPtrHash {
    std::size_t
    operator()(const std::pair<TopoNode::Ptr, TopoNode::Ptr> &p) const {
      return std::hash<TopoNode::Ptr>()(p.first) ^
             std::hash<TopoNode::Ptr>()(p.second);
    }
  };

  Eigen::Vector3i idx;
  planner_manager_->topo_graph_->getIndex(goal, idx);
  vector<TopoNode::Ptr> pre_nbrs;
  for (int i = -1; i <= 1; i++)
    for (int j = -1; j <= 1; j++)
      for (int k = -1; k <= 1; k++) {
        Eigen::Vector3i tmp_idx = idx;
        tmp_idx(0) = idx(0) + i;
        tmp_idx(1) = idx(1) + j;
        tmp_idx(2) = idx(2) + k;
        auto region = planner_manager_->topo_graph_->getRegionNode(tmp_idx);
        if (region) {
          for (auto &topo : region->topo_nodes_) {
            if (topo == ed_->next_goal_node_ || topo->is_viewpoint_ || preserved_anchors.count(topo))
              continue;
            pre_nbrs.emplace_back(topo);
          }
        }
      }
  if(!preserved_anchors.count(planner_manager_->topo_graph_->odom_node_) &&
      std::find(pre_nbrs.begin(),pre_nbrs.end(),planner_manager_->topo_graph_->odom_node_)==pre_nbrs.end())
    pre_nbrs.push_back(planner_manager_->topo_graph_->odom_node_);
  std::unordered_map<std::pair<TopoNode::Ptr, TopoNode::Ptr>,
                     vector<Eigen::Vector3f>, PairPtrHash>
      edge2insert;
  for(auto &edge:preserved)
    edge2insert.emplace(std::make_pair(ed_->next_goal_node_,edge.first),std::move(edge.second));
  if(!preserved.empty())ROS_INFO("Preserved %zu validated native connections for selected goal handoff",preserved.size());
  mutex edge2insert_mtx;
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &nbr : pre_nbrs) {
    vector<Eigen::Vector3f> path;
    int res = planner_manager_->topo_graph_->parallel_bubble_astar_->search(
        goal, nbr->center_, path, goal_connection_timeout);
    if (res == ParallelBubbleAstar::REACH_END &&
        planner_manager_->topo_graph_->parallel_bubble_astar_
            ->collisionCheck_shortenPath(path)) {
      edge2insert_mtx.lock();
      edge2insert.insert({std::make_pair(ed_->next_goal_node_, nbr), path});
      edge2insert_mtx.unlock();
    }
  }
  // Detach with the OLD position before moving the goal. Never retain edges
  // whose stored paths end at a previous TSP target when reconnection fails.
  // Native updateOdomNode intentionally retains outgoing odom edges only.
  // Such an incoming edge is absent from goal->neighbors_, so removeNode
  // alone cannot detach it when the same goal pointer moves to a new target.
  const auto odom=planner_manager_->topo_graph_->odom_node_;
  if(odom) {
    odom->neighbors_.erase(ed_->next_goal_node_);
    odom->paths_.erase(ed_->next_goal_node_);
    odom->weight_.erase(ed_->next_goal_node_);
    odom->unreachable_nbrs_.erase(ed_->next_goal_node_);
  }
  Eigen::Vector3i old_idx;
  planner_manager_->topo_graph_->getIndex(ed_->next_goal_node_->center_, old_idx);
  if (planner_manager_->topo_graph_->getRegionNode(old_idx))
    planner_manager_->topo_graph_->removeNode(ed_->next_goal_node_);
  ed_->next_goal_node_->center_ = goal;
  ed_->next_goal_node_->is_viewpoint_ = true;
  auto goal_region = planner_manager_->topo_graph_->getRegionNode(idx);
  if (goal_region) goal_region->topo_nodes_.insert(ed_->next_goal_node_);
  if (edge2insert.size() > 0) {
    for (auto &edge : edge2insert) {
      ed_->next_goal_node_->neighbors_.insert(edge.first.second);
      ed_->next_goal_node_->paths_.insert({edge.first.second, edge.second});
      double cost;
      planner_manager_->topo_graph_->parallel_bubble_astar_->calculatePathCost(
          edge.second, cost);
      ed_->next_goal_node_->weight_[edge.first.second] = cost;
      auto nbr = edge.first.second;
      nbr->neighbors_.insert(ed_->next_goal_node_);
      nbr->weight_[ed_->next_goal_node_] = cost;
      vector<Eigen::Vector3f> path = edge.second;
      std::reverse(path.begin(), path.end());
      nbr->paths_[ed_->next_goal_node_] = path;
    }
  }
}
} // namespace fast_planner
