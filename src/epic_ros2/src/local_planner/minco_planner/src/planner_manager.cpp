#include <stdexcept>
/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-28 14:48:50
 * @LastEditTime: 2023-12-30 15:02:15
 * @Description:
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved.
 */
// #include <fstream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <plan_manage/planner_manager.h>
#include <std_msgs/msg/int32.hpp>
#include <thread>
#include <chrono>
#include <visualization_msgs/msg/marker.hpp>

namespace fast_planner {
// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {}

FastPlannerManager::~FastPlannerManager() {
  lidar_map_interface_.reset();
  gcopter_viz_.reset();
  std::cout << "des manager" << std::endl;
}

void FastPlannerManager::printTimeCost(double time_threhold, double time_cost,
                                       string printInfo) {
  if (time_cost > time_threhold) {
    std::cout << "\033[31m " << printInfo << time_cost << " ms" << "\033[0m"
              << std::endl;
  } else {
    std::cout << "\033[32m " << printInfo << time_cost << " ms" << "\033[0m"
              << std::endl;
  }
}

void FastPlannerManager::initPlanModules(
    epic_ros::NodeHandle &nh, ParallelBubbleAstar::Ptr &parallel_path_finder,
    TopoGraph::Ptr &graph) {

  local_data_.traj_id_ = 0;

  lidar_map_interface_ = graph->lidar_map_interface_;
  nh.getParam("max_traj_len", max_traj_len_);
  nh.param("StraightSweepHorizon",straight_sweep_horizon_,0.);
  if(!std::isfinite(straight_sweep_horizon_) || straight_sweep_horizon_<0. || straight_sweep_horizon_>12.)
    throw std::invalid_argument("StraightSweepHorizon must be between 0 and 12 m");
  nh.getParam("lidar_perception/max_ray_length", max_ray_length);
  nh.getParam("lidar_perception/fov_up", fov_up);
  nh.getParam("lidar_perception/fov_down", fov_down);
  nh.getParam("lidar_perception/lidar_pitch", lidar_pitch);

  gcopter_viz_.reset(new Visualizer);
  gcopter_viz_->init(nh);
  gcopter_config_.reset(new GcopterConfig);
  gcopter_config_->init(nh);

  graph_visualizer_.reset(new GraphVisualizer);
  graph_visualizer_->init(nh);
  bubble_path_finder_.reset(new BubbleAstar);
  bubble_path_finder_->init(nh, lidar_map_interface_);
  topo_graph_ = graph;

  parallel_path_finder_ = parallel_path_finder;
  fast_searcher_.reset(new FastSearcher);
  fast_searcher_->init(topo_graph_, bubble_path_finder_);

  pos_sub = nh.subscribe("/quad_0/lidar_slam/odom", 10,
                         &FastPlannerManager::posCallback, this);
  goal_sub = nh.subscribe("/move_base_simple/goal", 10,
                          &FastPlannerManager::goalCallback, this);
  yaw_state_pub = nh.advertise<std_msgs::msg::Int32>("/quad_0/yaw_state", 10);
}

// test_gs
void FastPlannerManager::posCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {

  // 提取四元数
  double roll, pitch;
  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.pose.orientation, quat);

  // 将四元数转换为Euler角
  tf2::Matrix3x3(quat).getRPY(roll, pitch, local_data_.curr_yaw_);
}

void FastPlannerManager::goalCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
  // 提取四元数
  double roll, pitch;
  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.orientation, quat);

  // 将四元数转换为Euler角
  tf2::Matrix3x3(quat).getRPY(roll, pitch, local_data_.end_yaw_);
}

bool FastPlannerManager::recoverStartCorridor(const Eigen::Vector3d &start,
    const std::vector<Eigen::Vector3d> &guide, std::vector<Eigen::MatrixX4d> &polys, size_t minimum_join) {
  if(!gcopter_config_->startCorridorRecovery || polys.empty() || !start.allFinite() ||
      minimum_join>=polys.size())return false;
  const double hard=gcopter_config_->dilateRadiusHard;
  const double recovery_margin=gcopter_config_->startRecoveryMargin;
  const double clearance=lidar_map_interface_->getDisToOcc(start);
  if(!std::isfinite(clearance) || clearance<hard+recovery_margin+.05 ||
     start.z()<gcopter_config_->flightMinZ || start.z()>gcopter_config_->flightMaxZ)return false;
  auto inside=[](const Eigen::MatrixX4d &poly,const Eigen::Vector3d &point,double margin) {
    if(poly.rows()<4 || !poly.allFinite())return false;
    const auto norms=poly.leftCols<3>().rowwise().norm().eval();
    if(norms.minCoeff()<1.e-9)return false;
    return ((poly.leftCols<3>()*point+poly.col(3)).array()/norms.array()<=-margin).all();
  };
  if(minimum_join==0)for(const auto &poly:polys)if(inside(poly,start,1.e-6))return false;
  Eigen::Vector3d target=start;
  double nearest=2.01;
  for(const auto &point:guide) {
    const double distance=(point-start).norm();
    if(distance>=nearest)continue;
    for(const auto &poly:polys)if(inside(poly,point,.02)) {target=point;nearest=distance;break;}
  }
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(20);
  // Inscribed KD-clearance cubes connect the current boundary to native FIRI.
  // Prefer the normal comfort margin; a short recovery uses the configured
  // additional margin above HARD clearance. Every cube remains certified.
  // Every cube remains inside a sphere certified against the SAME collision map.
  for(double margin:{std::max(hard,gcopter_config_->dilateRadiusSoft),hard+recovery_margin}) {
    if(margin>gcopter_config_->dilateRadiusSoft+1.e-6)continue;
    Eigen::Vector3d center=start;
    std::vector<Eigen::MatrixX4d> prefix;
    size_t guide_index=0;double travelled=0.;
    for(int step=0;step<(minimum_join ? 64 : 24);++step) {
      if(std::chrono::steady_clock::now()>deadline)return false;
      const double distance=lidar_map_interface_->getDisToOcc(center);
      const double half=(distance-margin-.02)/std::sqrt(3.);
      if(!std::isfinite(half) || half<.02)break;
      Eigen::Vector3d lo=center-Eigen::Vector3d::Constant(half),hi=center+Eigen::Vector3d::Constant(half);
      lo.z()=std::max(lo.z(),gcopter_config_->flightMinZ);
      hi.z()=std::min(hi.z(),gcopter_config_->flightMaxZ);
      if((hi-lo).minCoeff()<.02)break;
      Eigen::MatrixX4d cube(6,4);
      cube<<1.,0.,0.,-hi.x(), -1.,0.,0.,lo.x(),
            0.,1.,0.,-hi.y(), 0.,-1.,0.,lo.y(),
            0.,0.,1.,-hi.z(), 0.,0.,-1.,lo.z();
      prefix.push_back(cube);
      for(int index=int(polys.size())-1;index>=int(minimum_join);--index) {
        if(std::chrono::steady_clock::now()>deadline)return false;
        if(polys[index].rows()<4 || !polys[index].allFinite() ||
           polys[index].leftCols<3>().rowwise().norm().minCoeff()<1.e-9)continue;
        if(geo_utils::overlap(cube,polys[index],.01)) {
          prefix.insert(prefix.end(),polys.begin()+index,polys.end());
          polys.swap(prefix);
          ROS_INFO("Start corridor recovered with KD-clearance connector; margin %.3f, hard %.3f",margin,hard);
          return true;
        }
      }
      // A disconnected departure already contains start. Follow only the
      // existing native guide to the far side of that gap; returning to the
      // first containing cell would recreate a zero-progress stop forever.
      if(minimum_join) {
        while(guide_index<guide.size() && (guide[guide_index]-center).norm()<1.e-3)++guide_index;
        if(guide_index==guide.size() || !guide[guide_index].allFinite())break;
        target=guide[guide_index];
      }
      const Eigen::Vector3d direction=target-center;
      if(direction.norm()<1.e-4)break;
      const double advance=std::min(.5*half,direction.norm());
      travelled+=advance;if(minimum_join && travelled>3.)break;
      center+=direction.normalized()*advance;
    }
  }
  return false;
}

bool FastPlannerManager::hasSafeTerminalHold() const {
  if(!gcopter_config_->safeTerminalHoldFallback || local_data_.minco_traj_.getPieceNum()==0)return false;
  const auto &traj=local_data_.minco_traj_;
  const double end=traj.getTotalDuration();
  if(!std::isfinite(end) || end<=0.)return false;
  const Eigen::Vector3d position=traj.getPos(end),velocity=traj.getVel(end);
  const Eigen::Vector3d acceleration=traj.getAcc(end),jerk=traj.getJer(end);
  if(!position.allFinite() || !velocity.allFinite() || !acceleration.allFinite() || !jerk.allFinite() ||
     velocity.norm()>.01 || acceleration.norm()>.05 || jerk.norm()>.1 ||
     position.z()<gcopter_config_->flightMinZ || position.z()>gcopter_config_->flightMaxZ)return false;
  // The server holds its final near-zero-derivative sample after the polynomial.
  // Clearance padding also covers that tiny endpoint sampling difference.
  const double clearance=lidar_map_interface_->getDisToOcc(position);
  return std::isfinite(clearance) && clearance>=gcopter_config_->dilateRadiusHard+.1;
}

bool FastPlannerManager::checkTrajHeight() const {
  const double lo=gcopter_config_->flightMinZ-1.e-5, hi=gcopter_config_->flightMaxZ+1.e-5;
  if(lo<-1.e5 && hi>1.e5)return true; // Disabled for existing profiles.
  double minimum=std::numeric_limits<double>::infinity(),maximum=-minimum;
  for(const auto &piece:local_data_.minco_traj_) {
    const double duration=piece.getDuration();
    if(!std::isfinite(duration) || duration<=0. || !piece.getCoeffMat().allFinite())return false;
    // Find coordinate extrema in normalized time, avoiding badly scaled
    // boundary-crossing polynomials for very short/long MINCO pieces.
    Eigen::VectorXd normalized=piece.getCoeffMat().row(2).transpose();
    Eigen::VectorXd derivative(7);
    for(int i=0;i<8;++i)normalized(i)*=std::pow(duration,7-i);
    if(!normalized.allFinite())return false;
    for(int i=0;i<7;++i)derivative(i)=(7-i)*normalized(i);
    std::set<double> critical{0.,1.};
    if(derivative.cwiseAbs().maxCoeff()>1.e-12) {
      const auto roots=RootFinder::solvePolynomial(derivative,0.,1.,1.e-8);
      critical.insert(roots.begin(),roots.end());
    }
    for(double u:critical) {
      const double z=piece.getPos(u*duration).z();
      if(!std::isfinite(z))return false;
      minimum=std::min(minimum,z);maximum=std::max(maximum,z);
    }
  }
  if(minimum<=lo || maximum>=hi) {
    ROS_WARN("Trajectory height range [%.6f, %.6f] outside [%.6f, %.6f]",minimum,maximum,lo,hi);
    return false;
  }
  return true;
}

bool FastPlannerManager::checkTrajVelocity(double trajectory_age) {
  auto traj = local_data_.minco_traj_;
  double duration = local_data_.duration_;
  double curr_time = trajectory_age >= 0.0 ? trajectory_age :
      std::max(0., (epic_ros::Time::now() - local_data_.start_time_).toSec());
  while (curr_time < duration) {
    Vector3d curr_vel = traj.getVel(curr_time);
    if (curr_vel.norm() > gcopter_config_->maxVelMag + 1.0) {
      ROS_WARN("Trajectory velocity %.3f exceeds %.3f at t=%.3f; initial velocity %.3f",
               curr_vel.norm(),gcopter_config_->maxVelMag+1.,curr_time,traj.getVel(0.).norm());
      return false;
    }
    curr_time += 0.3;
  }
  return true;
}

bool FastPlannerManager::checkTrajCollision(double &collision_time, double trajectory_age) {
  collision_time = 0.;
  if (local_data_.minco_traj_.getPieceNum() == 0) return true;
  PointType target;
  PointVector nearest_point;
  vector<float> PointDist;

  auto traj = local_data_.minco_traj_;
  double duration = local_data_.duration_;
  double curr_time = trajectory_age >= 0.0 ? trajectory_age :
      std::max(0., (epic_ros::Time::now() - local_data_.start_time_).toSec());
  const double current_age = curr_time;
  collision_time = std::max(0., duration - current_age);
  Vector3d last_sphere_cen_;
  if (curr_time > duration) {
    collision_time = 0.;
    return true;
  }

  last_sphere_cen_ = traj.getPos(curr_time);
  double last_radius_ = lidar_map_interface_->getDisToOcc(last_sphere_cen_) -
                        gcopter_config_->dilateRadiusHard;
  while (curr_time < duration) {
    Vector3d curr_pos = traj.getPos(curr_time);
    if ((curr_pos - last_sphere_cen_).norm() < last_radius_) {
      curr_time += 0.05;
      continue;
    }
    // 超出了上一个球的范围, 更新一个球
    last_radius_ = lidar_map_interface_->getDisToOcc(curr_pos) -
                   gcopter_config_->dilateRadiusHard;
    last_sphere_cen_ = curr_pos;
    if (last_radius_ < 0) {
      collision_time = curr_time - current_age;
      return false;
    }
  }
  return true;
}

size_t FastPlannerManager::connectedCorridorPrefix(const std::vector<Eigen::MatrixX4d> &corridors) {
  if(corridors.empty())return 0;
  for(const auto &poly:corridors)
    if(poly.rows()<4 || !poly.allFinite() || (poly.leftCols<3>().rowwise().norm().array()<1.e-8).any())return 0;
  Eigen::Vector3d inner;
  if(!geo_utils::findInterior(corridors.front(),inner))return 0;
  size_t count=1;
  while(count<corridors.size() && geo_utils::overlap(corridors[count-1],corridors[count],1.e-2))++count;
  return count;
}

void FastPlannerManager::addFlightHeightConstraints(std::vector<Eigen::MatrixX4d> &corridors) const {
  if(gcopter_config_->flightMinZ<=-1.e5 && gcopter_config_->flightMaxZ>=1.e5)return;
  const double lo=gcopter_config_->flightMinZ+gcopter_config_->heightPlanningMargin;
  const double hi=gcopter_config_->flightMaxZ-gcopter_config_->heightPlanningMargin;
  for(auto &poly:corridors) {
    const Eigen::Index rows=poly.rows();poly.conservativeResize(rows+2,4);
    poly.row(rows)<<0.,0.,1.,-hi;
    poly.row(rows+1)<<0.,0.,-1.,lo;
  }
}

std::vector<Eigen::Vector3d> FastPlannerManager::boundedGuidePrefix(
    const std::vector<Eigen::Vector3f> &path,double max_length) {
  std::vector<Eigen::Vector3d> prefix;
  if(path.size()<2 || !std::isfinite(max_length) || max_length<=0. || !path.front().allFinite())return prefix;
  prefix.push_back(path.front().cast<double>());
  double used=0.;
  for(size_t i=1;i<path.size();++i) {
    if(!path[i].allFinite())return {};
    const Eigen::Vector3d next=path[i].cast<double>(),delta=next-prefix.back();
    const double length=delta.norm();if(length<1.e-8)continue;
    if(used+length>max_length) {
      const double remaining=max_length-used;
      if(remaining>1.e-8)prefix.push_back(prefix.back()+delta*(remaining/length));
      break;
    }
    prefix.push_back(next);used+=length;
    if(used>=max_length-1.e-8)break;
  }
  if(prefix.size()<2)return {};
  return prefix;
}

double FastPlannerManager::guideHorizon(const vector<Eigen::Vector3f> &path,
                                       double ordinary,double straight) {
  if(!std::isfinite(straight) || straight<=ordinary || straight>12. || path.size()<2)return ordinary;
  const auto candidate=boundedGuidePrefix(path,straight);
  if(candidate.size()<2)return ordinary;
  const Eigen::Vector3d delta=candidate.back()-candidate.front();
  if(delta.norm()<=ordinary+.1)return ordinary;
  double previous=0.;
  for(const auto &point:candidate) {
    const double fraction=(point-candidate.front()).dot(delta)/delta.squaredNorm();
    if(fraction<previous-1.e-4 || fraction>1.0001 ||
       (point-candidate.front()-fraction*delta).norm()>.05)return ordinary;
    previous=fraction;
  }
  return straight;
}

bool FastPlannerManager::planExploreTraj(const vector<Eigen::Vector3f> &path,
                                         bool is_static) {

  epic_ros::Time start = epic_ros::Time::now();

  const double horizon=guideHorizon(path,max_traj_len_,straight_sweep_horizon_);
  if(horizon>max_traj_len_)ROS_INFO("Straight sweep horizon %.1fm; ordinary horizon %.1fm",horizon,max_traj_len_);
  auto path_shorten=boundedGuidePrefix(path,horizon);
  if(path_shorten.size()<2)return false;
  // Bound work by ARC LENGTH. A nearby goal can still require a warehouse-
  // sized detour; Euclidean proximity must not expand the local horizon.
  const bool use_shorten_path=(path_shorten.back()-path.back().cast<double>()).norm()>1.e-5;
  if(use_shorten_path) {
    const Eigen::Vector3d fwd_dir=path_shorten.back()-path_shorten[path_shorten.size()-2];
    if(fwd_dir.head<2>().squaredNorm()>fwd_dir.z()*fwd_dir.z())
      local_data_.end_yaw_=atan2(fwd_dir.y(),fwd_dir.x());
  }

  // 从小到大
  Eigen::Vector3f min_bd, max_bd;
  for (int i = 0; i < 3; i++) {
    min_bd[i] = path_shorten[0][i];
    max_bd[i] = path_shorten[0][i];
  }
  for (const Eigen::Vector3d &waypoint : path_shorten) {
    for (int i = 0; i < 3; i++) {
      if (waypoint[i] < min_bd[i]) {
        min_bd[i] = waypoint[i];
      }
      if (waypoint[i] > max_bd[i]) {
        max_bd[i] = waypoint[i];
      }
    }
  }
  for (int i = 0; i < 2; i++) {
    min_bd[i] = (min_bd[i] - 3.0); // range
    max_bd[i] = (max_bd[i] + 3.0);
  }
  min_bd[2] -= 1.0;
  max_bd[2] += 1.0;

  PointVector Searched_Points;
  lidar_map_interface_->boxSearch(min_bd, max_bd, Searched_Points);

  // 降采样
  std::vector<Eigen::Vector3d> surf_points;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud_origin->points = Searched_Points;
  sor.setInputCloud(cloud_origin);
  sor.setLeafSize(0.2, 0.2, 0.2);
  sor.filter(*cloud_tmp);

  surf_points.reserve(cloud_tmp->points.size());
  for (const pcl::PointXYZ &point : cloud_tmp->points) {
    surf_points.emplace_back(point.x, point.y, point.z);
  }

  epic_ros::Time point_process_end_stamp = epic_ros::Time::now();

  std::vector<Eigen::MatrixX4d> hPolys; // 多面体飞行走廊

  sfc_gen::convexCover(gcopter_viz_, path_shorten, surf_points,
                       min_bd.cast<double>(), max_bd.cast<double>(), 7.0,
                       gcopter_config_->corridor_size, hPolys, 1e-6,
                       gcopter_config_->dilateRadiusSoft);
  // Add the navigation band AFTER FIRI's obstacle-clearance erosion.
  // Eroding a narrow navigation box by DilateRadiusSoft makes it empty.
  // Obstacle query bounds and the original collision halfspaces stay intact.
  // Optional inner buffer accommodates soft-constraint solver error. The
  // final whole-polynomial check retains the ORIGINAL hard flight bounds.
  addFlightHeightConstraints(hPolys);
  Eigen::Matrix<double, 3, 4> iniState;
  Eigen::Matrix<double, 3, 4> finState;
  const epic_ros::Time boundary_time = epic_ros::Time::now();
  double time_now = (boundary_time - local_data_.start_time_).toSec();
  if (is_static || local_data_.minco_traj_.getPieceNum() == 0) {
    // TSP、更新地图等会阻塞里程计回调函数，导致这里的数据不准，所以只有static才用
    iniState << local_data_.curr_pos_, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  } else {
    time_now =
        time_now > local_data_.duration_ ? local_data_.duration_ : time_now;
    Eigen::Vector3d current_pose = local_data_.minco_traj_.getPos(time_now);
    Eigen::Vector3d curr_vel = local_data_.minco_traj_.getVel(time_now);
    Eigen::Vector3d curr_acc = local_data_.minco_traj_.getAcc(time_now);
    Eigen::Vector3d curr_jerk = local_data_.minco_traj_.getJer(time_now);
    iniState << current_pose, curr_vel, curr_acc, curr_jerk;
  }

  Eigen::Vector4d bh;
  bh << iniState.topLeftCorner<3, 1>(), 1.0;
  auto startIndex=[&]() {
    for(int i=int(hPolys.size())-1;i>=0;--i)
      if((hPolys[i]*bh).maxCoeff()<=-1.e-6)return i;
    return -1;
  };
  int start_idx=startIndex();
  const bool start_recovered=start_idx<0 && recoverStartCorridor(iniState.col(0),path_shorten,hPolys);
  if(start_recovered)start_idx=startIndex();
  if (start_idx == -1) {
    ROS_ERROR("current position not in corridor: boundary [%.3f %.3f %.3f], KD clearance %.3f, guide-start gap %.3f, cells %zu",
        iniState(0,0),iniState(1,0),iniState(2,0),
        lidar_map_interface_->getDisToOcc(Eigen::Vector3d(iniState.col(0))),
        (path_shorten.front()-iniState.col(0)).norm(),hPolys.size());
    // No valid corridor: reject the candidate; FSM retains a safe old path or lands.
    return false;
  }
  if (start_idx != 0) {
    hPolys.erase(hPolys.begin(), hPolys.begin() + start_idx);
  }
  sfc_gen::shortCut(hPolys);

  // epic_ros::Duration(1.0).sleep();

  epic_ros::Time hpoly_gen_end = boundary_time;

  if (hPolys.size() < 2) {
    cout << "hPolys size < 2" << endl;
    return false;
  }
  // Height intersection may disconnect even the final FIRI pair. Validate
  // every pair before GCOPTER, then use the native safe-prefix stop policy.
  size_t connected=connectedCorridorPrefix(hPolys);
  if(connected>0 && connected<hPolys.size() &&
      recoverStartCorridor(iniState.col(0),path_shorten,hPolys,connected)) {
    sfc_gen::shortCut(hPolys);
    connected=connectedCorridorPrefix(hPolys);
  }
  if(connected==0) {
    ROS_ERROR("No feasible initial corridor after height intersection");
    return false;
  }
  if (connected < hPolys.size()) {
    ROS_WARN("Disconnected corridor tail: retain %zu/%zu certified polytopes",connected,hPolys.size());
    Eigen::Vector3d inner;
    if(!geo_utils::findInterior(hPolys[connected-1],inner))return false;
    finState << inner, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero();
    hPolys.resize(connected);
    gcopter_viz_->visualizePolytope(hPolys, true);
  } else {
    finState << path_shorten.back(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    gcopter_viz_->visualizePolytope(hPolys);
  }
  gcopter_viz_->visualizeRoute(path);

  gcopter::GCOPTER_PolytopeSFC gcopter;
  Eigen::VectorXd magnitudeBounds(9);
  Eigen::VectorXd penaltyWeights(9);
  Eigen::VectorXd physicalParams(6);
  magnitudeBounds(0) = gcopter_config_->maxVelMag;
  magnitudeBounds(1) = gcopter_config_->maxBdrMag;
  magnitudeBounds(2) = gcopter_config_->maxTiltAngle;
  magnitudeBounds(3) = gcopter_config_->minThrust;
  magnitudeBounds(4) = gcopter_config_->maxThrust;
  magnitudeBounds(5) = gcopter_config_->maxAccMag;
  magnitudeBounds(6) = gcopter_config_->maxJerkMag;
  magnitudeBounds(7) = gcopter_config_->maxVelUp;
  magnitudeBounds(8) = gcopter_config_->maxVelDown;
  penaltyWeights(0) = (gcopter_config_->chiVec)[0];
  penaltyWeights(1) = (gcopter_config_->chiVec)[1];
  penaltyWeights(2) = (gcopter_config_->chiVec)[2];
  penaltyWeights(3) = (gcopter_config_->chiVec)[3];
  penaltyWeights(4) = (gcopter_config_->chiVec)[4];
  penaltyWeights(5) = penaltyWeights(6) = 1.0e5;
  penaltyWeights(7) = penaltyWeights(8) = 1.0e5;
  physicalParams(0) = gcopter_config_->vehicleMass;
  physicalParams(1) = gcopter_config_->gravAcc;
  physicalParams(2) = gcopter_config_->horizDrag;
  physicalParams(3) = gcopter_config_->vertDrag;
  physicalParams(4) = gcopter_config_->parasDrag;
  physicalParams(5) = gcopter_config_->speedEps;
  const int quadratureRes = gcopter_config_->integralIntervs;

  if (!gcopter.setup(
          gcopter_config_->weightT, gcopter_config_->dilateRadiusSoft, iniState,
          finState, hPolys, gcopter_config_->lengthPerPiece, gcopter_config_->smoothingEps,
          quadratureRes, magnitudeBounds, penaltyWeights, physicalParams)) {
    std::cout << "\n\n\n\n\n\n\n\nsetup failed!" << std::endl;

    return false;
  }
  auto local_data_backup = local_data_;
  local_data_.minco_traj_.clear();
  double time_lb;
  calculateTimelb(path_shorten, local_data_.curr_yaw_, local_data_.end_yaw_,
                  time_lb);
  cout << "lower_bd = " << time_lb << endl;
  const auto solve_started=std::chrono::steady_clock::now();
  if (std::isinf(gcopter.optimize(local_data_.minco_traj_,
                                  gcopter_config_->relCostTol, time_lb,
                                  gcopter_config_->optimizerTimeBudgetMs))) {
    std::cout << "optimize failed!" << std::endl;
    local_data_ = local_data_backup;
    return false;
  } else {
    local_data_.duration_ = local_data_.minco_traj_.getTotalDuration();
  }
  epic_ros::NodeHandle candidate_nh;
  bool strict_candidate_collision;
  candidate_nh.param("StrictCandidateCollision",strict_candidate_collision,false);
  auto candidateStatus=[&]() {
    double time=10.;
    if(!checkTrajCollision(time,0.) && (time<1. || start_recovered || strict_candidate_collision)) {
      ROS_WARN("Reject candidate collision at %.3f s; strict full-trajectory check %d",time,int(strict_candidate_collision));
      return 1;
    }
    if(!checkTrajHeight())return 2;
    if(!checkTrajVelocity(0.))return 3;
    return 0;
  };
  int candidate_status=candidateStatus();
  const double elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-solve_started).count();
  const double remaining_ms=gcopter_config_->optimizerTimeBudgetMs-elapsed_ms;
  if((candidate_status==2 || candidate_status==3 ||
      (candidate_status==1 && strict_candidate_collision)) && remaining_ms>5.) {
    // Re-optimize from a different INITIAL seed, keeping the exact boundary
    // PVAJ, goal and corridor. Both solves share the original wall budget.
    ROS_WARN("Retry invalid MINCO candidate with slower seed; remaining budget %.1f ms",remaining_ms);
    local_data_.minco_traj_.clear();
    if(std::isfinite(gcopter.optimize(local_data_.minco_traj_,gcopter_config_->relCostTol,time_lb,remaining_ms,2.))) {
      local_data_.duration_=local_data_.minco_traj_.getTotalDuration();
      candidate_status=candidateStatus();
      if(candidate_status==0)ROS_INFO("MINCO retry recovered candidate with unchanged boundary and checks");
    }
  }
  if(candidate_status!=0) {
    std::cout<<(candidate_status==1 ? "check traj collision failed" : candidate_status==2 ? "check traj height failed" : "check traj velocity failed")<<std::endl;
    local_data_=local_data_backup;return false;
  }
  if (local_data_.minco_traj_.getPieceNum() > 0) {
    if (!YawTrajOpt(local_data_.curr_yaw_, local_data_.end_yaw_, is_static,
                    use_shorten_path)) {
      cout << "yaw_traj_opt failed!" << endl;
      local_data_ = local_data_backup;
      return false;
    }
    gcopter_viz_->visualize(local_data_.minco_traj_,
                            gcopter_config_->maxVelMag);
  } else {
    local_data_ = local_data_backup;
    std::cout << "traj empty!" << std::endl;
    return false;
  }
  epic_ros::Time optimize_end_stamp = epic_ros::Time::now();
  double trajOptimize_time =
      (optimize_end_stamp - hpoly_gen_end).toSec() * 1000;
  local_data_.traj_id_ += 1;
  local_data_.start_time_ = hpoly_gen_end;
  local_data_.start_pos_ = path_shorten.front();
  local_data_.duration_ = local_data_.minco_traj_.getTotalDuration();
  return true;
}

void FastPlannerManager::angleLimite(double &angle) {
  while (angle > M_PI) {
    angle -= (M_PI * 2);
  }
  while (angle < -M_PI) {
    angle += (M_PI * 2);
  }
}

bool FastPlannerManager::YawTrajOpt(double &start_yaw, double &end_yaw,
                                    bool is_static, bool use_shorten_path) {
  Eigen::Matrix3d iniStateYaw, finStateYaw;
  Eigen::MatrixXd wpsYaw;
  Eigen::VectorXd opt_times_Yaw;
  double end_yaw_temp;
  double init_yaw;
  double time_now = (epic_ros::Time::now() - local_data_.start_time_).toSec();
  double yaw_sp, yaw_sv(0.0), yaw_sa(0.0), yaw_ep(end_yaw), yaw_ev(0.0),
      yaw_ea(0.0);

  if (is_static) {
    yaw_sp = start_yaw;
  } else {
    time_now = time_now > local_data_.minco_yaw_traj_.getTotalDuration()
                   ? local_data_.minco_yaw_traj_.getTotalDuration()
                   : time_now;
    yaw_sp = local_data_.minco_yaw_traj_.getPos(time_now).x();
    yaw_sv = local_data_.minco_yaw_traj_.getVel(time_now).x();
    yaw_sa = local_data_.minco_yaw_traj_.getAcc(time_now).x();
  }
  angleLimite(yaw_sp);
  static double yaw_dur = 0.3;
  // double yaw_dur = local_data_.duration_ / 12.0;
  double fwd_time = gcopter_config_->yaw_time_fwd;
  vector<double> look_fwd_wp;
  look_fwd_wp.push_back(yaw_sp);
  for (double t = yaw_dur; t < local_data_.duration_ + yaw_dur; t += yaw_dur) {
    if (t > local_data_.duration_) {
      double delta = local_data_.end_yaw_ - look_fwd_wp.back();
      angleLimite(delta);
      look_fwd_wp.push_back(look_fwd_wp.back() + delta);
      break;
    }
    Eigen::Vector3d p_c = local_data_.minco_traj_.getPos(t);
    double t_fwd = t + fwd_time;
    Eigen::Vector3d p_f =
        local_data_.minco_traj_.getPos(min(t_fwd, local_data_.duration_));
    Eigen::Vector2d dir = p_f.head(2) - p_c.head(2);
    if (dir.norm() < fabs(p_f.z() - p_c.z()) || dir.norm() < 0.05) {
      // if (p_f.z() - p_c.z() > 0) {
      //   look_fwd_wp.push_back(look_fwd_wp.back());
      // } else
      // if (dir.norm() > 0.1) {
      //   double yaw = atan2(dir.y(), dir.x()) + min(M_PI, yaw_dur *
      //   gcopter_config_->yaw_max_vel); double delta_yaw = yaw -
      //   look_fwd_wp.back(); angleLimite(delta_yaw);
      //   look_fwd_wp.push_back(look_fwd_wp.back() + delta_yaw);
      // } else
      look_fwd_wp.push_back(
          look_fwd_wp.back() +
          min(M_PI, yaw_dur * gcopter_config_->yaw_max_vel * 0.6));
    } else {
      double yaw = atan2(dir.y(), dir.x());
      double delta_yaw = yaw - look_fwd_wp.back();
      angleLimite(delta_yaw);
      look_fwd_wp.push_back(look_fwd_wp.back() + delta_yaw);
    }
  }

  vector<double> wp;
  double delta_yaw_max = gcopter_config_->yaw_max_vel * yaw_dur;
  std::random_device rd;
  std::mt19937 gen(rd()); // 使用Mersenne Twister作为随机数引擎

  std::normal_distribution<double> dist(0, 2e-3);
  bool turn2end = false;

  for (int i = 0; i < look_fwd_wp.size(); i++) {
    if (i == 0) {
      wp.push_back(look_fwd_wp[i]);
      continue;
    }
    double last_yaw = wp.back();
    double diff2end_yaw = local_data_.end_yaw_ - last_yaw;
    angleLimite(diff2end_yaw);
    double time2end_yaw = abs(diff2end_yaw) / gcopter_config_->yaw_max_vel;
    double time_last = local_data_.duration_ - i * yaw_dur;
    double next_yaw;
    if (time_last <= time2end_yaw)
      next_yaw = local_data_.end_yaw_ + dist(gen);
    else
      next_yaw = look_fwd_wp[i] + dist(gen);

    double delta_yaw = next_yaw - last_yaw;
    angleLimite(delta_yaw);
    if (delta_yaw < 0 && delta_yaw < -delta_yaw_max) {
      delta_yaw = -delta_yaw_max;
    } else if (delta_yaw > 0 && delta_yaw > delta_yaw_max) {
      delta_yaw = delta_yaw_max;
    }
    wp.push_back(wp.back() + delta_yaw);
  }
  double delta2end = local_data_.end_yaw_ - wp.back();
  angleLimite(delta2end);
  local_data_.end_yaw_ = wp.back() + delta2end;
  yaw_ep = local_data_.end_yaw_;
  iniStateYaw << Eigen::Vector3d(yaw_sp, 0.0, 0.0),
      Eigen::Vector3d(yaw_sv, 0.0, 0.0), Eigen::Vector3d(yaw_sa, 0.0, 0.0);
  finStateYaw << Eigen::Vector3d(yaw_ep, 0.0, 0.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero();

  gcopter::GCOPTER_PolytopeSFC gcopter_yaw;
  if (!gcopter_yaw.setup_yaw(gcopter_config_->yaw_rho_vis,
                             gcopter_config_->integralIntervs)) {
    cout << "setup_yaw failed!" << endl;
    return false;
  }
  int pieceNUM = wp.size() - 2;
  if (pieceNUM <= 1) {
    opt_times_Yaw.resize(2);
    opt_times_Yaw[0] = local_data_.duration_ / 2.0;
    opt_times_Yaw[1] = local_data_.duration_ / 2.0;
    wpsYaw.resize(3, 1);
    wpsYaw(0, 0) = (wp[0] + wp[1]) / 2.0;
    wpsYaw(1, 0) = 0.0;
    wpsYaw(2, 0) = 0.0;
    pieceNUM = 2;
  } else {
    // 干掉最后一个值
    opt_times_Yaw.resize(wp.size() - 2);
    for (int i = 0; i < wp.size() - 3; i++) {
      opt_times_Yaw[i] = yaw_dur;
    }
    opt_times_Yaw[wp.size() - 3] =
        local_data_.duration_ - (wp.size() - 3) * yaw_dur;
    wpsYaw.resize(3, wp.size() - 3);
    for (int i = 1; i < wp.size() - 2; i++) {
      wpsYaw(0, i - 1) = wp[i];
      wpsYaw(1, i - 1) = 0.0;
      wpsYaw(2, i - 1) = 0.0;
    }
  }
  // double dur_yaw = 0.0;
  // for (int i = 0; i < opt_times_Yaw.size(); i++) {
  //   dur_yaw += opt_times_Yaw[i];
  // }
  // cout << "dur_p= " << local_data_.duration_ << " dur_yaw= " << dur_yaw <<
  // endl; cout << "start yaw = " << iniStateYaw.col(0).transpose() << endl;
  // cout << "end yaw = " << finStateYaw.col(0).transpose() << endl;
  // cout << "wpsYaw = " << endl;
  // for (int i = 0; i < wpsYaw.cols(); i++) {
  //   cout << wpsYaw.col(i).transpose()(0) << " ";
  // }
  // cout << endl;

  local_data_.minco_yaw_traj_.clear();
  if (std::isinf(gcopter_yaw.optimize_yaw(iniStateYaw, finStateYaw, pieceNUM,
                                          wpsYaw, opt_times_Yaw,
                                          local_data_.minco_yaw_traj_))) {
    std::cout << "optimize yaw failed!" << std::endl;
    return false;
  }
  return true;
}

bool FastPlannerManager::flyToSafeRegion(bool is_static) {
  Eigen::Vector3f min_bd, max_bd;
  for (int i = 0; i < 3; i++) {
    min_bd[i] = topo_graph_->odom_node_->center_[i] - 2.0;
    max_bd[i] = topo_graph_->odom_node_->center_[i] + 2.0;
  }
  PointVector Searched_Points;
  lidar_map_interface_->boxSearch(min_bd, max_bd, Searched_Points);
  std::vector<Eigen::Vector3d> surf_points;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud_origin->points = Searched_Points;
  sor.setInputCloud(cloud_origin);
  sor.setLeafSize(0.2, 0.2, 0.2);
  sor.filter(*cloud_tmp);

  surf_points.reserve(cloud_tmp->points.size());
  for (const pcl::PointXYZ &point : cloud_tmp->points) {
    surf_points.emplace_back(point.x, point.y, point.z);
  }
  Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
  bd(0, 0) = 1.0;
  bd(1, 0) = -1.0;
  bd(2, 1) = 1.0;
  bd(3, 1) = -1.0;
  bd(4, 2) = 1.0;
  bd(5, 2) = -1.0;
  bd(0, 3) =
      -(std::min(topo_graph_->odom_node_->center_(0) + 2.0f,
                 lidar_map_interface_->lp_->global_map_max_boundary_[0]));
  bd(1, 3) = std::max(topo_graph_->odom_node_->center_(0) - 2.0f,
                      lidar_map_interface_->lp_->global_box_min_boundary_[0]);
  bd(2, 3) =
      -(std::min(topo_graph_->odom_node_->center_(1) + 2.0f,
                 lidar_map_interface_->lp_->global_map_max_boundary_[1]));
  bd(3, 3) = std::max(topo_graph_->odom_node_->center_(1) - 2.0f,
                      lidar_map_interface_->lp_->global_box_min_boundary_[1]);
  bd(4, 3) =
      -(std::min(topo_graph_->odom_node_->center_(2) + 1.0f,
                 lidar_map_interface_->lp_->global_map_max_boundary_[2]));
  bd(5, 3) = std::max(topo_graph_->odom_node_->center_(2) - 1.0f,
                      lidar_map_interface_->lp_->global_box_min_boundary_[2]);
  Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(
      surf_points[0].data(), 3, surf_points.size());
  Eigen::MatrixX4d hp;
  firi::firi(bd, pc, topo_graph_->odom_node_->center_.cast<double>(),
             topo_graph_->odom_node_->center_.cast<double>(),
             hp); // 计算出包含a和b的凸包
  std::vector<Eigen::MatrixX4d> hPolys;
  hPolys.push_back(hp);
  hPolys.push_back(hp);
  Eigen::Vector3d inner;
  geo_utils::findInterior(hp, inner);
  Eigen::Vector4d bh;
  double time_now = (epic_ros::Time::now() - local_data_.start_time_).toSec();
  Eigen::Matrix<double, 3, 4> iniState;
  Eigen::Vector3d dir =
      (inner - topo_graph_->odom_node_->center_.cast<double>()).normalized();
  if (is_static || local_data_.minco_traj_.getPieceNum() == 0) {
    // TSP、更新地图等会阻塞里程计回调函数，导致这里的数据不准，所以只有static才用
    iniState << local_data_.curr_pos_, dir * 0.2, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero();
    // iniState << topo_graph_->odom_node_->center_, local_data_.curr_vel_,
    // Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  } else {
    time_now =
        time_now > local_data_.duration_ ? local_data_.duration_ : time_now;
    Eigen::Vector3d current_pose = local_data_.minco_traj_.getPos(time_now);
    Eigen::Vector3d curr_vel = local_data_.minco_traj_.getVel(time_now);
    Eigen::Vector3d curr_acc = local_data_.minco_traj_.getAcc(time_now);
    Eigen::Vector3d curr_jerk = local_data_.minco_traj_.getJer(time_now);
    // iniState << current_pose, curr_vel, curr_acc, curr_jerk;
    iniState << current_pose, curr_vel, curr_acc, curr_jerk;
  }
  Eigen::Matrix<double, 3, 4> finState;
  epic_ros::Time hpoly_gen_end = epic_ros::Time::now();
  // iniState << topo_graph_->odom_node_->center_, local_data_.curr_vel_,
  // Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  finState << inner, dir, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  bh << iniState.topLeftCorner<3, 1>(), 1.0;
  int start_idx = -1;
  for (int i = hPolys.size() - 1; i >= 0; i--) {
    Eigen::MatrixX4d hp = hPolys[i];
    if ((((hp * bh).array() > -1.0e-6).cast<int>().sum() <= 0)) {
      start_idx = i;
      break;
    }
  }
  if (start_idx == -1) {
    ROS_ERROR("current position not in corridor");
    return false;
  }
  if (start_idx != 0) {
    hPolys.erase(hPolys.begin(), hPolys.begin() + start_idx);
  }
  sfc_gen::shortCut(hPolys);

  // epic_ros::Duration(1.0).sleep();

  if (hPolys.size() < 2) {
    cout << "hPolys size < 2" << endl;
    return false;
  }
  gcopter_viz_->visualizePolytope(hPolys);
  gcopter::GCOPTER_PolytopeSFC gcopter;
  Eigen::VectorXd magnitudeBounds(9);
  Eigen::VectorXd penaltyWeights(9);
  Eigen::VectorXd physicalParams(6);
  magnitudeBounds(0) = gcopter_config_->maxVelMag;
  magnitudeBounds(1) = gcopter_config_->maxBdrMag;
  magnitudeBounds(2) = gcopter_config_->maxTiltAngle;
  magnitudeBounds(3) = gcopter_config_->minThrust;
  magnitudeBounds(4) = gcopter_config_->maxThrust;
  magnitudeBounds(5) = gcopter_config_->maxAccMag;
  magnitudeBounds(6) = gcopter_config_->maxJerkMag;
  magnitudeBounds(7) = gcopter_config_->maxVelUp;
  magnitudeBounds(8) = gcopter_config_->maxVelDown;
  penaltyWeights(0) = (gcopter_config_->chiVec)[0] * 2.0;
  penaltyWeights(1) = (gcopter_config_->chiVec)[1] / 2.0;
  penaltyWeights(2) = (gcopter_config_->chiVec)[2] / 2.0;
  penaltyWeights(3) = (gcopter_config_->chiVec)[3] / 2.0;
  penaltyWeights(4) = (gcopter_config_->chiVec)[4] / 2.0;
  physicalParams(0) = gcopter_config_->vehicleMass;
  physicalParams(1) = gcopter_config_->gravAcc;
  physicalParams(2) = gcopter_config_->horizDrag;
  physicalParams(3) = gcopter_config_->vertDrag;
  physicalParams(4) = gcopter_config_->parasDrag;
  physicalParams(5) = gcopter_config_->speedEps;
  const int quadratureRes = gcopter_config_->integralIntervs;

  if (!gcopter.setup(
          gcopter_config_->WeightSafeT, gcopter_config_->dilateRadiusSoft,
          iniState, finState, hPolys, gcopter_config_->lengthPerPiece, gcopter_config_->smoothingEps,
          quadratureRes, magnitudeBounds, penaltyWeights, physicalParams)) {
    std::cout << "\n\n\n\n\n\n\n\nsetup failed!" << std::endl;

    return false;
  }
  auto local_data_backup = local_data_;
  local_data_.minco_traj_.clear();
  if (std::isinf(gcopter.optimize(local_data_.minco_traj_,
                                  gcopter_config_->relCostTol, 0.0,
                                  gcopter_config_->optimizerTimeBudgetMs))) {
    std::cout << "optimize failed!" << std::endl;
    local_data_ = local_data_backup;
    return false;
  }
  if (local_data_.minco_traj_.getPieceNum() > 0) {
    // ROS_INFO_STREAM(
    // "local_data_.minco_traj_.getPieceNum(): " <<
    // local_data_.minco_traj_.getPieceNum());
    gcopter_viz_->visualize(local_data_.minco_traj_,
                            gcopter_config_->maxVelMag);
  } else {
    local_data_ = local_data_backup;
    std::cout << "traj empty!" << std::endl;
    return false;
  }
  epic_ros::Time optimize_end_stamp = epic_ros::Time::now();
  local_data_.traj_id_ += 1;
  local_data_.start_time_ = hpoly_gen_end;
  local_data_.start_pos_ = topo_graph_->odom_node_->center_.cast<double>();
  local_data_.duration_ = local_data_.minco_traj_.getTotalDuration();
  return true;
}

void FastPlannerManager::polyTraj2ROSMsg(epic_ros2::msg::PolyTraj &poly_msg,
                                         const epic_ros::Time &start_time) {
  Eigen::VectorXd durs = local_data_.minco_traj_.getDurations();
  int piece_num = local_data_.minco_traj_.getPieceNum();
  poly_msg.drone_id = 0;
  poly_msg.traj_id = local_data_.traj_id_;
  poly_msg.start_time = start_time;
  poly_msg.order = 7;
  poly_msg.duration.resize(piece_num);
  poly_msg.coef_x.resize(8 * piece_num);
  poly_msg.coef_y.resize(8 * piece_num);
  poly_msg.coef_z.resize(8 * piece_num);
  for (int i = 0; i < piece_num; ++i) {
    poly_msg.duration[i] = durs(i);
    Eigen::Matrix<double, 3, 8> cMat =
        local_data_.minco_traj_.pieces[i].getCoeffMat();
    int i6 = i * 8;
    for (int j = 0; j < 8; j++) {
      poly_msg.coef_x[i6 + j] = cMat(0, j);
      poly_msg.coef_y[i6 + j] = cMat(1, j);
      poly_msg.coef_z[i6 + j] = cMat(2, j);
    }
  }
}

void FastPlannerManager::polyYawTraj2ROSMsg(epic_ros2::msg::PolyTraj &poly_msg,
                                            const epic_ros::Time &start_time) {
  Eigen::VectorXd durs = local_data_.minco_yaw_traj_.getDurations();
  int piece_num = local_data_.minco_yaw_traj_.getPieceNum();
  poly_msg.drone_id = 0;
  poly_msg.traj_id = local_data_.traj_id_;
  poly_msg.start_time = start_time;
  poly_msg.order = 5;
  poly_msg.duration.resize(piece_num);
  poly_msg.coef_x.resize(6 * piece_num);
  poly_msg.coef_y.resize(6 * piece_num);
  poly_msg.coef_z.resize(6 * piece_num);
  for (int i = 0; i < piece_num; ++i) {
    poly_msg.duration[i] = durs(i);
    Eigen::Matrix<double, 3, 6> cMat =
        local_data_.minco_yaw_traj_.pieces[i].getCoeffMat();
    int i6 = i * 6;
    for (int j = 0; j < 6; j++) {
      poly_msg.coef_x[i6 + j] = cMat(0, j);
      poly_msg.coef_y[i6 + j] = cMat(1, j);
      poly_msg.coef_z[i6 + j] = cMat(2, j);
    }
  }
}

void FastPlannerManager::calculateTimelb(
    const vector<Eigen::Vector3d> &path2next_goal, const double &current_yaw,
    const double &goal_yaw, double &time_lb) {
  double start2fwd = 0.0, fwd2end = 0.0;
  if (path2next_goal.size() == 2) {
    Eigen::Vector3d diff = path2next_goal.back() - path2next_goal.front();
    if (pow(diff.x(), 2) + pow(diff.y(), 2) >= pow(diff.z(), 2) &&
        (diff.squaredNorm() - pow(diff.z(), 2) > 0.01)) {
      double fwd_yaw = atan2(diff.y(), diff.x());
      start2fwd = fwd_yaw - current_yaw;
      angleLimite(start2fwd);
      fwd2end = goal_yaw - fwd_yaw;
      angleLimite(fwd2end);

    } else {
      double diff = goal_yaw - current_yaw;
      angleLimite(diff);
      time_lb = fabs(diff) / gcopter_config_->yaw_max_vel;
      return;
    }
  } else {
    Eigen::Vector3d diff = path2next_goal[1] - path2next_goal[0];
    if (pow(diff.x(), 2) + pow(diff.y(), 2) >= pow(diff.z(), 2)) {
      double fwd_yaw = atan2(diff.y(), diff.x());
      start2fwd = fwd_yaw - current_yaw;
      angleLimite(start2fwd);
    } else {
      start2fwd = 0.0;
    }
    diff = path2next_goal[path2next_goal.size() - 1] -
           path2next_goal[path2next_goal.size() - 2];
    if (pow(diff.x(), 2) + pow(diff.y(), 2) >= pow(diff.z(), 2)) {
      double fwd_yaw = atan2(diff.y(), diff.x());
      fwd2end = fwd_yaw - goal_yaw;
      angleLimite(fwd2end);
    } else {
      fwd2end = 0.0;
    }
  }

  time_lb = fabs(start2fwd) / gcopter_config_->yaw_max_vel +
            fabs(fwd2end) / gcopter_config_->yaw_max_vel;
  return;
}
} // namespace fast_planner
