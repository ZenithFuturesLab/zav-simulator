/*
If you need to replace with other point cloud data structures, please
re-implement the following interfaces:

1. getDisToOcc: Returns the distance from the specified point to the nearest
obstacle in the map.
2. KNN: Nearest neighbor search. boxSearch: Region search.
3. updateCloudMapOdometry: Point cloud map update. 
4. LIOInterfaceData::Ptr ld: Current frame world coordinate point clouds and
lidar-odometry.

If you need to integrate EPIC with Lidar SLAM algorithm and shares
memory, thread mutual exclusion should be noted.
*/
#include <visualization_msgs/msg/marker.hpp>
#include <lidar_map/lidar_map.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <array>
#include <cstring>
#include <map>
namespace fast_planner {

double LIOInterface::getDisToOcc(const PointType &pt) {
  const auto measure = [&] {
    const float squared_distance = ikd_Tree_map.nearestSquaredDistance(pt, 10.0);
    return std::isfinite(squared_distance) ? sqrt(squared_distance) : 10.0;
  };
  const uint64_t revision = ld_ ? ld_->obstacle_revision.load(std::memory_order_acquire) : 0;
  if (!revision) return measure();

  // Only identical XYZ bits in the same immutable map can share a result.
  // A hash collision replaces an entry. It never approximates a clearance.
  struct Entry {
    uint64_t revision;
    std::array<uint32_t,3> xyz;
    double distance;
  };
  static thread_local std::array<Entry,8192> cache{};
  std::array<uint32_t,3> xyz;
  static_assert(sizeof(float)==sizeof(uint32_t));
  std::memcpy(xyz.data(),pt.data,3*sizeof(float));
  uint32_t hash=xyz[0]*0x9e3779b1u + xyz[1]*0x85ebca77u + xyz[2]*0xc2b2ae3du;
  hash=(hash^(hash>>16))*0x85ebca77u;
  auto &entry=cache[(hash^(hash>>13)) % cache.size()];
  if (entry.revision==revision && entry.xyz==xyz) return entry.distance;
  entry={revision,xyz,measure()};
  return entry.distance;
}
double LIOInterface::getDisToOcc(const Eigen::Vector3d &pt) {
  PointType p;
  p.x = pt.x();
  p.y = pt.y();
  p.z = pt.z();
  return getDisToOcc(p);
}
double LIOInterface::getDisToOcc(const Eigen::Vector3f &pt) {
  PointType p;
  p.x = pt.x();
  p.y = pt.y();
  p.z = pt.z();
  return getDisToOcc(p);
}
void LIOInterface::KNN(const PointType &pt, int k, PointVector &pts,
                       vector<float> &dis) {
  ikd_Tree_map.Nearest_Search(pt, k, pts, dis, 10.0);
}
void LIOInterface::boxSearch(const Eigen::Vector3f &min_bd,
                             const Eigen::Vector3f &max_bd, PointVector &pts) {
  BoxPointType boxpoint;
  for (int i = 0; i < 3; i++) {
    boxpoint.vertex_min[i] = min_bd(i);
    boxpoint.vertex_max[i] = max_bd(i);
  }
  ikd_Tree_map.Box_Search(boxpoint, pts);
}
// A distant hit enters the obstacle map immediately. Retire it only when
// repeated, closer lidar scans actually pass through it from different poses.
struct HitContradictions {
  struct Vote {Eigen::Vector3f first_sensor;size_t count=0,last_scan=0;};
  size_t serial=0;
  std::map<std::array<int,3>,Vote> votes;
};

void LIOInterface::reconcileHits(const Eigen::Vector3f &sensor,const pcl::PointCloud<PointType>::ConstPtr &scan) {
  if (!ld_->contradicted_hits_) ld_->contradicted_hits_=std::make_shared<HitContradictions>();
  auto &history=*ld_->contradicted_hits_;
  const size_t serial=++history.serial;
  if ((serial%2)!=0) return;
  for (auto it=history.votes.begin();it!=history.votes.end();)
    if (serial-it->second.last_scan>24) it=history.votes.erase(it);else ++it;
  // A registered scan spans a tenth of a second. Only attribute traversed
  // rays to one sensor origin when translation during that scan is small.
  if (ld_->lidar_vel_.norm()>1.2f || scan->empty()) return;
  constexpr int cols=180,rows=90;
  const auto index=[](const Eigen::Vector3f &ray) {
    constexpr float scale=90.f/float(M_PI);
    const int col=int(std::floor(scale*std::atan2(ray.y(),ray.x())))+90;
    const int row=int(std::floor(scale*std::atan2(ray.z(),std::hypot(ray.x(),ray.y()))))+45;
    return std::array<int,2>{(col+cols)%cols,std::clamp(row,0,rows-1)};
  };
  std::array<float,rows*cols> depth;
  depth.fill(std::numeric_limits<float>::infinity());
  for (const auto &hit:*scan) {
    const Eigen::Vector3f ray=hit.getVector3fMap()-sensor;
    if (!ray.allFinite()) continue;
    const auto bin=index(ray);
    auto &d=depth[bin[1]*cols+bin[0]];
    d=std::min(d,ray.norm());
  }
  const float range=std::min(8.f,float(.6*lp_->persistent_hit_range_));
  const Eigen::Vector3f span=Eigen::Vector3f::Constant(range);
  PointVector nearby,pending,retired;
  boxSearch(sensor-span,sensor+span,nearby);
  for (const auto &hit:nearby) {
    const Eigen::Vector3f point=hit.getVector3fMap(),ray=point-sensor;
    const float r=ray.norm();
    if (r>range || r<.5f) continue;
    const auto bin=index(ray);
    const int x=bin[0],y=bin[1];
    if (!y || y==rows-1) continue;
    const float centre=depth[y*cols+x];
    if (!std::isfinite(centre)) continue;
    const float horizontal=std::min(depth[y*cols+(x+cols-1)%cols],depth[y*cols+(x+1)%cols]);
    const float vertical=std::min(depth[(y-1)*cols+x],depth[(y+1)*cols+x]);
    if (!std::isfinite(horizontal) || !std::isfinite(vertical)) continue;
    const float nearest=std::min({centre,horizontal,vertical});
    const Eigen::Vector3i cell=(point*10.f).array().floor().cast<int>();
    const std::array<int,3> key{cell.x(),cell.y(),cell.z()};
    if (nearest<=r+.30f) {
      if (std::abs(nearest-r)<.16f) history.votes.erase(key);
      continue;
    }
    auto &v=history.votes[key];
    if (!v.count) v.first_sensor=sensor;
    if (v.last_scan!=serial) {++v.count;v.last_scan=serial;}
    if (v.count>=3 && (sensor-v.first_sensor).norm()>=.12f) pending.push_back(hit);
  }
  if (pending.empty()) return;
  pcl::KdTreeFLANN<PointType> current;
  current.setInputCloud(scan);
  std::vector<int> ids(1);std::vector<float> squared(1);
  for (const auto &hit:pending) {
    if (current.nearestKSearch(hit,1,ids,squared)!=1 || squared[0]<=.16f*.16f) continue;
    retired.push_back(hit);
    const Eigen::Vector3i cell=(hit.getVector3fMap()*10.f).array().floor().cast<int>();
    history.votes.erase({cell.x(),cell.y(),cell.z()});
  }
  if (!retired.empty()) ikd_Tree_map.Delete_Points(retired);
}

void LIOInterface::updateCloudMapOdometry(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr &odom_) {
  const std::lock_guard<std::mutex> writer(ld_->scan_writer);
  ld_->map_update = true;
  Eigen::Vector3f lidar_pos_(odom_->pose.pose.position.x,
                             odom_->pose.pose.position.y,
                             odom_->pose.pose.position.z);
  Eigen::Vector3f lidar_vel_(odom_->twist.twist.linear.x,
                             odom_->twist.twist.linear.y,
                             odom_->twist.twist.linear.z);
  ld_->lidar_pose_ = lidar_pos_;
  // EPIC plans in world coordinates; ROS Odometry twist is in the child frame.
  ld_->lidar_vel_ = lp_->odom_twist_in_world_ ? lidar_vel_ : Eigen::Quaternionf(odom_->pose.pose.orientation.w,
                                      odom_->pose.pose.orientation.x,
                                      odom_->pose.pose.orientation.y,
                                      odom_->pose.pose.orientation.z) * lidar_vel_;
  Eigen::AngleAxisf y_axis_angle(M_PI / 180.0 * lp_->lidar_pitch_,
                                 Eigen::Vector3f::UnitY());
  Eigen::Quaternionf q_y(y_axis_angle);
  ld_->lidar_q_ = Eigen::Quaternionf(odom_->pose.pose.orientation.w,
                                     odom_->pose.pose.orientation.x,
                                     odom_->pose.pose.orientation.y,
                                     odom_->pose.pose.orientation.z) *
                  q_y;

  pcl::fromROSMsg(*msg, ld_->lidar_cloud_);
  // PointVector pcl_map = points.points;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(0.1, 0.1, 0.1);
  vg.setInputCloud(ld_->lidar_cloud_.makeShared());
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  vg.filter(*filtered_points);
  auto &pcl_map=filtered_points->points;
  // Persistent obstacle hits share the range over which the explorer can
  // establish observed air. The full current scan remains available for
  // raycasting. Close hits enter the archive on their first observation.
  const double range=lp_->persistent_hit_range_;
  if (range>0.) {
    pcl_map.erase(std::remove_if(pcl_map.begin(),pcl_map.end(),[&](const auto &point) {
      return (point.getVector3fMap()-lidar_pos_).squaredNorm()>range*range;
    }),pcl_map.end());
  }

  if (pcl_map.empty())
    return;

  ld_->obstacle_revision.store(0,std::memory_order_release);
  if (ld_->first_map_flag_) {
    // this->ikd_Tree_map(0.3,0.6,0.2);
    this->ikd_Tree_map.set_downsample_param(0.1);
    this->ikd_Tree_map.Build(pcl_map);
    ld_->first_map_flag_ = false;
  } else {
    if (range>0.) {
      const auto &q=odom_->pose.pose.orientation;
      const Eigen::Vector3f sensor=lidar_pos_+Eigen::Quaternionf(q.w,q.x,q.y,q.z)*lp_->sensor_offset_body_;
      reconcileHits(sensor,filtered_points);
    }
    this->ikd_Tree_map.Add_Points(pcl_map, true);
  }
  // Also distinguish new map instances and reused object addresses on worker threads.
  static std::atomic<uint64_t> next_revision{1};
  ld_->obstacle_revision.store(next_revision.fetch_add(1,std::memory_order_relaxed),
                               std::memory_order_release);
}

} // namespace fast_planner