#include <visualization_msgs/msg/marker.hpp>
#include <lidar_map/lidar_map.h>
#include <pcl/filters/voxel_grid.h>
#include <stdexcept>
namespace fast_planner {
namespace {
using BoxInterval = std::pair<double, double>;

bool clipSegment(const Eigen::Vector3d &start, const Eigen::Vector3d &delta,
                 const Eigen::Vector3f &lower, const Eigen::Vector3f &upper,
                 BoxInterval &interval) {
  interval = {0., 1.};
  for (int axis = 0; axis < 3; ++axis) {
    if (delta[axis] == 0.) {
      if (start[axis] < lower[axis] || start[axis] > upper[axis]) return false;
      continue;
    }
    const double lower_crossing = (lower[axis] - start[axis]) / delta[axis];
    const double upper_crossing = (upper[axis] - start[axis]) / delta[axis];
    interval.first = std::max(interval.first, std::min(lower_crossing, upper_crossing));
    interval.second = std::min(interval.second, std::max(lower_crossing, upper_crossing));
    if (interval.first > interval.second) return false;
  }
  return true;
}
}

LIOInterface::LIOInterface() {}

LIOInterface::~LIOInterface() {}

bool LIOInterface::IsPathInBox(const vector<Eigen::Vector3f> &path) const {
  if (path.size() < 2 || !path.front().allFinite()) return false;
  vector<BoxInterval> coverage;
  if (lp_->box_num_ > 1) coverage.reserve(lp_->box_num_);
  for (size_t i = 1; i < path.size(); ++i) {
    if (!path[i].allFinite()) return false;
    const Eigen::Vector3d start = path[i - 1].cast<double>();
    const Eigen::Vector3d delta = path[i].cast<double>() - start;
    BoxInterval interval;
    for (int j = 0; j < lp_->dead_area_num_; ++j) {
      if (clipSegment(start, delta, lp_->dead_area_min_boundary_vec_[j],
                      lp_->dead_area_max_boundary_vec_[j], interval)) return false;
    }
    // The whole segment must be covered by the union of the allowed boxes.
    coverage.clear();
    double covered = 0.;
    for (int j = 0; j < lp_->box_num_; ++j) {
      if (!clipSegment(start, delta, lp_->global_box_min_boundary_vec_[j],
                       lp_->global_box_max_boundary_vec_[j], interval)) continue;
      if (interval.first == 0. && interval.second == 1.) {
        covered = 1.;
        break;
      }
      if (lp_->box_num_ > 1) coverage.push_back(interval);
    }
    if (covered == 1.) continue;
    std::sort(coverage.begin(), coverage.end());
    for (const auto &span : coverage) {
      if (span.first > covered) break;
      covered = std::max(covered, span.second);
    }
    if (covered < 1.) return false;
  }
  return true;
}

void LIOInterface::init(epic_ros::NodeHandle &nh) {
  lp_.reset(new LIOInterfaceParam);
  ld_.reset(new LIOInterfaceData);
  ikd_Tree_map.setMap_ikdtree(this);
  ikd_Tree_map.Set_delete_criterion_param(0.3);
  ikd_Tree_map.Set_balance_criterion_param(0.6);
  string odom_topic, cloud_topic;
  nh.param("lidar_perception/odom_twist_in_world", lp_->odom_twist_in_world_, false);
  std::vector<double> sensor_offset;
  nh.param("lidar_perception/sensor_offset_body",sensor_offset,std::vector<double>{});
  if (sensor_offset.size()==3)
    lp_->sensor_offset_body_={float(sensor_offset[0]),float(sensor_offset[1]),float(sensor_offset[2])};
  nh.getParam("odometry_topic", odom_topic);
  nh.getParam("cloud_topic", cloud_topic);
  nh.param("box_num", lp_->box_num_, 0);
  if (lp_->box_num_ < 1)
    throw std::invalid_argument("EPIC requires box_num >= 1 in its map configuration");
  const auto boxCorner = [&](const string &parameter) -> Eigen::Vector3f {
    vector<double> values;
    nh.getParam(parameter, values);
    if (values.size() != 3 ||
        !std::all_of(values.begin(), values.end(), [](double value) {
          return std::abs(value) <= std::numeric_limits<float>::max();
        })) throw std::invalid_argument("EPIC requires three finite coordinates for " + parameter);
    return {float(values[0]), float(values[1]), float(values[2])};
  };
  nh.getParam("lidar_perception/lidar_pitch", lp_->lidar_pitch_);
  for (int i = 0; i < lp_->box_num_; i++) {
    const auto tmp1 = boxCorner("box_" + to_string(i) + "/down");
    const auto tmp2 = boxCorner("box_" + to_string(i) + "/up");
    Eigen::Vector3f min, max;
    for (int i = 0; i < 3; i++) {
      min(i) = fmin(tmp1(i), tmp2(i));
      max(i) = fmax(tmp1(i), tmp2(i));
    }
    lp_->global_box_min_boundary_vec_.emplace_back(min);
    lp_->global_box_max_boundary_vec_.emplace_back(max);
  }
  nh.param("dead_area_num", lp_->dead_area_num_, 0);
  for (int i = 0; i < lp_->dead_area_num_; i++) {
    const auto tmp1 = boxCorner("dead_" + to_string(i) + "/down");
    const auto tmp2 = boxCorner("dead_" + to_string(i) + "/up");
    Eigen::Vector3f min, max;
    for (int i = 0; i < 3; i++) {
      min(i) = fmin(tmp1(i), tmp2(i));
      max(i) = fmax(tmp1(i), tmp2(i));
    }
    lp_->dead_area_min_boundary_vec_.emplace_back(min);
    lp_->dead_area_max_boundary_vec_.emplace_back(max);
  }

  lp_->global_box_max_boundary_ = lp_->global_box_min_boundary_vec_[0];
  lp_->global_box_min_boundary_ = lp_->global_box_max_boundary_vec_[0];
  for (auto &vec : lp_->global_box_min_boundary_vec_) {
    lp_->global_box_min_boundary_.x() =
        vec.x() < lp_->global_box_min_boundary_.x()
            ? vec.x()
            : lp_->global_box_min_boundary_.x();
    lp_->global_box_min_boundary_.y() =
        vec.y() < lp_->global_box_min_boundary_.y()
            ? vec.y()
            : lp_->global_box_min_boundary_.y();
    lp_->global_box_min_boundary_.z() =
        vec.z() < lp_->global_box_min_boundary_.z()
            ? vec.z()
            : lp_->global_box_min_boundary_.z();
  }
  for (auto &vec : lp_->global_box_max_boundary_vec_) {
    lp_->global_box_max_boundary_.x() =
        vec.x() > lp_->global_box_max_boundary_.x()
            ? vec.x()
            : lp_->global_box_max_boundary_.x();
    lp_->global_box_max_boundary_.y() =
        vec.y() > lp_->global_box_max_boundary_.y()
            ? vec.y()
            : lp_->global_box_max_boundary_.y();
    lp_->global_box_max_boundary_.z() =
        vec.z() > lp_->global_box_max_boundary_.z()
            ? vec.z()
            : lp_->global_box_max_boundary_.z();
  }
  lp_->global_map_max_boundary_ = lp_->global_box_max_boundary_;
  lp_->global_map_min_boundary_ = lp_->global_box_min_boundary_;
  cout << "max boundary: " << lp_->global_map_max_boundary_ << endl;
  cout << "min boundary: " << lp_->global_map_min_boundary_ << endl;
  nh.param("lidar_perception/fov_up", lp_->fov_up, -0.1);
  nh.param("lidar_perception/fov_down", lp_->fov_down, -0.1);
  nh.param("lidar_perception/fov_viewpoint_up", lp_->fov_vp_up, -0.1);
  nh.param("lidar_perception/fov_viewpoint_down", lp_->fov_vp_down, -0.1);
  nh.getParam("lidar_perception/max_ray_length", lp_->max_ray_length_);
  ld_->first_map_flag_ = true;
  // update_trigger_puber_ =
  //     nh.advertise<std_msgs::msg::Empty>("/lio_interface/map_updated", 1);
  // cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(
  //     nh, cloud_topic, 5));
  // odom_sub_.reset(
  //     new message_filters::Subscriber<nav_msgs::msg::Odometry>(nh.native().get(), odom_topic, rmw_qos_profile_sensor_data));
  // sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
  //     SyncPolicyCloudOdom(1), *cloud_sub_, *odom_sub_));
  // sync_cloud_odom_->registerCallback(
  //     boost::bind(&LIOInterface::updateCloudMapOdometry, this, _1, _2));
}

} // namespace fast_planner
