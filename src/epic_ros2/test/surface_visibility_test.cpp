#include <frontier_manager/frontier_manager.h>
#include <cassert>
#include <pcl_conversions/pcl_conversions.h>

struct SurfaceVisibilityRegression {
  static void run(FrontierManager &m) {
    auto node=m.nh_.native();
    auto mode=[&](bool enabled) {
      node->set_parameter(rclcpp::Parameter("FrontierManager.measured_surface_visibility",enabled));
    };
    PointVector targets;
    for(int y=100;y<=101;++y)for(int z=7;z<=8;++z) {
      PointType p;m.idx2pos({100,y,z},p);targets.push_back(p);
      ByteArrayRaw key;m.pos2bytes(p,key);m.frtd_.label_map_[key]=FRONTIER_DIS;
      PointType front;m.idx2pos({101,y,z},front);
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
    add_plane(plane_x);
    auto cluster=make_shared<ClusterInfo>();
    m.compute_cluster_info(targets,vector<Eigen::Vector3f>(4,Eigen::Vector3f::UnitX()),cluster);
    m.graph_->odom_node_=make_shared<TopoNode>();
    m.graph_->odom_node_->center_=cluster->center_+Eigen::Vector3f(5,0,0);
    m.graph_->odom_node_->yaw_=0.;
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
    // A second physical plane inside the local voxel band must STILL occlude.
    add_plane(plane_x+.25f);
    assert(!score(1));
    mode(false);
    std::cout<<"PASS: adjacent self-surface voxel, front/back sign, and genuine intervening wall\n";
  }
};

int main(int argc,char **argv) {
  epic_ros::init(argc,argv,"surface_visibility_regression");epic_ros::NodeHandle nh;
  auto lio=make_shared<LIOInterface>();lio->init(nh);
  auto graph=make_shared<TopoGraph>();FrontierManager manager;manager.init(nh,lio,graph);
  SurfaceVisibilityRegression::run(manager);rclcpp::shutdown();
}
