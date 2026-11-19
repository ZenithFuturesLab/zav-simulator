#include <frontier_manager/frontier_manager.h>
#include <cassert>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>

struct FrontierClusteringRegression {
  static void run(FrontierManager &manager) {
    manager.frtp_.cluster_min_size_=.4f;
    manager.frtp_.cluster_minmum_point_num_=4;
    auto point=[&](int y,int z) {
      PointType p;manager.idx2pos(Eigen::Vector3i(65,y,z),p);
      ByteArrayRaw key;manager.pos2bytes(p,key);
      manager.frtd_.label_map_[key]=FRONTIER_DIS;
      manager.frtd_.frt_map_[key]=Eigen::Vector3f::UnitX();return p;
    };
    PointType a=point(18,4),b=point(19,4),c=point(18,5),d=point(19,5);
    vector<ClusterInfo::Ptr> created;vector<int> removed;
    PointVector first{a,b};manager.update_updating_aabb(first);
    manager.cluster_frts(first,created,removed);
    assert(manager.cluster_list_.empty());assert(manager.unclustered_frts_.size()==2);
    assert(!manager.hasInspectionFrontiers()); // Below the native cluster threshold.
    // The next scan supplies two neighbours plus a duplicate. Real clustering
    // must receive all four unique points, not drop each sub-threshold update.
    PointVector second{a,c,d};manager.update_updating_aabb(second);
    manager.cluster_frts(second,created,removed);
    assert(manager.cluster_list_.size()==1);assert(manager.cluster_list_.front()->cells_.size()==4);
    assert(manager.unclustered_frts_.empty());assert(manager.hasInspectionFrontiers());
    PointVector all{a,b,c,d};
    for(const auto&p:all) {ByteArrayRaw key;manager.pos2bytes(p,key);manager.frtd_.label_map_[key]=DENSE;}
    manager.update_updating_aabb(all);manager.cluster_frts({},created,removed);
    assert(manager.cluster_list_.empty());assert(!manager.hasInspectionFrontiers());
    // Four initially isolated points are unassigned, not permanently lost.
    PointVector isolated{point(25,4),point(30,4),point(35,4),point(40,4)};
    manager.update_updating_aabb(isolated);manager.cluster_frts(isolated,created,removed);
    assert(manager.unclustered_frts_.size()==4);assert(!manager.hasInspectionFrontiers());
    PointVector neighbours{point(25,5),point(26,4),point(26,5)};
    manager.update_updating_aabb(neighbours);manager.cluster_frts(neighbours,created,removed);
    assert(manager.cluster_list_.size()==1);assert(manager.cluster_list_.front()->cells_.size()==4);
    assert(manager.unclustered_frts_.size()==3);
    // Ignoring a horizontal surface is a target filter, never an observed
    // label that may erase neighbouring native wall frontiers.
    manager.cluster_list_.clear();manager.unclustered_frts_.clear();manager.unclustered_norms_.clear();
    manager.frtp_.target_max_abs_normal_z_=.3f;
    manager.frtp_.cluster_min_size_=.3f; // Keep fixture away from a float threshold.
    PointVector horizontal{point(45,4),point(46,4),point(45,5),point(46,5)};
    for(const auto&p:horizontal){ByteArrayRaw key;manager.pos2bytes(p,key);manager.frtd_.frt_map_[key]=Eigen::Vector3f::UnitZ();}
    manager.update_updating_aabb(horizontal);manager.cluster_frts(horizontal,created,removed);
    assert(!manager.hasInspectionFrontiers());
    for(const auto&p:horizontal)assert(manager.get_state(p)==FRONTIER_DIS);
    PointVector vertical{point(50,4),point(51,4),point(50,5),point(51,5)};
    manager.update_updating_aabb(vertical);manager.cluster_frts(vertical,created,removed);
    assert(manager.hasInspectionFrontiers());
    manager.cluster_list_.clear();manager.unclustered_frts_.clear();manager.unclustered_norms_.clear();
    manager.frtp_.floor_range_frontiers_=true;
    manager.lidar_map_interface_->ld_->lidar_pose_.z()=2.f;
    PointType step;step.x=3.;step.y=0.;step.z=2.2;
    assert(manager.isFloorRangeFrontier(step,Eigen::Vector3f::UnitZ(),FRONTIER_DIS));
    step.z=3.;
    assert(!manager.isFloorRangeFrontier(step,Eigen::Vector3f::UnitZ(),FRONTIER_DIS));
    assert(!manager.isFloorRangeFrontier(step,-Eigen::Vector3f::UnitZ(),FRONTIER_DIS));
    manager.frtp_.ascending_floor_frontiers_=true;
    step.z=1.2;
    assert(manager.isFloorRangeFrontier(step,Eigen::Vector3f::UnitZ(),FRONTIER_DIR));
    step.z=0.;
    assert(!manager.isFloorRangeFrontier(step,Eigen::Vector3f::UnitZ(),FRONTIER_DIR));
    manager.frtp_.ascending_floor_frontiers_=false;
    PointVector floor_range;
    for(int x=65;x<=66;++x)for(int y=45;y<=46;++y) {
      PointType p;manager.idx2pos({x,y,1},p);ByteArrayRaw key;manager.pos2bytes(p,key);
      manager.frtd_.label_map_[key]=FRONTIER_DIS;manager.frtd_.frt_map_[key]=Eigen::Vector3f::UnitZ();
      assert(manager.isFloorRangeFrontier(p,Eigen::Vector3f::UnitZ(),FRONTIER_DIS));
      assert(!manager.isFloorRangeFrontier(p,Eigen::Vector3f::UnitZ(),FRONTIER_DIR));
      floor_range.push_back(p);
    }
    manager.update_updating_aabb(floor_range);manager.cluster_frts(floor_range,created,removed);
    assert(manager.hasInspectionFrontiers());
    auto floor_cluster=manager.cluster_list_.front();
    const auto center=floor_cluster->center_;
    for(int x=50;x<=80;++x)for(int y=30;y<=60;++y) {
      PointType p;manager.idx2pos({x,y,1},p);ByteArrayRaw key;manager.pos2bytes(p,key);
      if(manager.get_state(p)!=FRONTIER_DIS)manager.frtd_.label_map_[key]=DENSE;
    }
    manager.graph_->odom_node_=make_shared<TopoNode>();
    manager.graph_->odom_node_->center_=center+Eigen::Vector3f(-8,0,1.7);
    manager.lidar_map_interface_->ld_->lidar_pose_=manager.graph_->odom_node_->center_;
    manager.frtp_.good_observation_trust_length_=8.f;
    // At 1.7m clearance a small floor patch cannot be seen from radii <=3m
    // with the actual 22deg mount / -5deg viewpoint FOV. It must stay pending,
    // rather than being called observed or forcing a zero-progress viewpoint.
    manager.vpp_.lidar_pitch_=22.f;
    manager.vpp_.fov_down_=-5.f*M_PI/180.f;
    manager.vpp_.fov_up_=48.f*M_PI/180.f;
    floor_cluster->vp_clusters_.clear();
    for(float offset:{2.f,3.f}) {
      ViewpointCluster vp;vp.center_=center+Eigen::Vector3f(offset,0,1.7);
      vp.vps_.emplace_back(vp.center_.x(),vp.center_.y(),vp.center_.z());vp.distance_=4.;
      floor_cluster->vp_clusters_.push_back(vp);
    }
    manager.selectBestViewpoint(floor_cluster);
    assert(!floor_cluster->is_reachable_ && manager.hasInspectionFrontiers());
    for(const auto&p:floor_range)assert(manager.get_state(p)==FRONTIER_DIS);
    floor_cluster->vp_clusters_.clear();
    for(float offset:{-4.f,4.f,6.f}) {
      ViewpointCluster vp;vp.center_=center+Eigen::Vector3f(offset,0,1.7);
      vp.vps_.emplace_back(vp.center_.x(),vp.center_.y(),vp.center_.z());vp.distance_=4.;
      floor_cluster->vp_clusters_.push_back(vp);
    }
    manager.selectBestViewpoint(floor_cluster);
    assert(floor_cluster->is_reachable_ && floor_cluster->best_vp_.x()>center.x());
    const auto available=floor_cluster->vp_clusters_;
    const auto preferred=floor_cluster->best_vp_;
    manager.deferViewpoint(preferred,15.);
    manager.selectBestViewpoint(floor_cluster);
    assert(floor_cluster->is_reachable_ && (floor_cluster->best_vp_-preferred).norm()>1.f);
    manager.deferViewpoint(floor_cluster->best_vp_,15.);
    manager.deferViewpoint(center+Eigen::Vector3f(-4,0,1.7),15.);
    manager.selectBestViewpoint(floor_cluster);
    assert(!floor_cluster->is_reachable_ && manager.hasInspectionFrontiers());
    for(const auto&p:floor_range)assert(manager.get_state(p)==FRONTIER_DIS);
    for(auto &entry:manager.deferred_viewpoints_)entry.second=-1.;
    floor_cluster->vp_clusters_=available;
    manager.selectBestViewpoint(floor_cluster);
    assert(floor_cluster->is_reachable_ && (floor_cluster->best_vp_-preferred).norm()<1.e-4f);
    for(const auto&p:floor_range){ByteArrayRaw key;manager.pos2bytes(p,key);manager.frtd_.label_map_[key]=FRONTIER_DIR;}
    manager.update_updating_aabb(floor_range);manager.cluster_frts(floor_range,created,removed);
    assert(!manager.hasInspectionFrontiers());
    // Measured floor cells just outside trusted range can be disconnected
    // from dense cells because of oblique beams. They must still seed progress.
    auto seed_lio=make_shared<LIOInterface>();seed_lio->init(manager.nh_);
    pcl::PointCloud<PointType> seed_cloud;
    for(float x=8.2f;x<=8.8f;x+=.1f)for(float y=-.5f;y<=.5f;y+=.1f)seed_cloud.emplace_back(x,y,0.f);
    auto cloud_msg=make_shared<sensor_msgs::msg::PointCloud2>();pcl::toROSMsg(seed_cloud,*cloud_msg);
    auto seed_odom=make_shared<nav_msgs::msg::Odometry>();seed_odom->pose.pose.position.z=2.1;seed_odom->pose.pose.orientation.w=1.;
    seed_lio->updateCloudMapOdometry(cloud_msg,seed_odom);
    // Surface targets require actual nearby points, independently of world
    // height. Reject a ghost layer while retaining its original labels.
    auto support_lio=make_shared<LIOInterface>();support_lio->init(manager.nh_);
    support_lio->updateCloudMapOdometry(cloud_msg,seed_odom);
    auto support_graph=make_shared<TopoGraph>();FrontierManager supported;
    supported.init(manager.nh_,support_lio,support_graph);
    supported.frtp_.floor_range_frontiers_=true;
    supported.frtp_.max_frontier_support_distance_=.55f;
    supported.frtp_.cluster_min_size_=.3f;
    supported.frtp_.cluster_minmum_point_num_=4;
    auto layer=[&](float z) {
      Eigen::Vector3i base;supported.pos2idx(Eigen::Vector3f(8.2f,-.2f,z),base);
      PointVector points;
      for(int x=0;x<2;++x)for(int y=0;y<2;++y) {
        PointType p;supported.idx2pos(base+Eigen::Vector3i(x,y,0),p);
        ByteArrayRaw key;supported.pos2bytes(p,key);
        supported.frtd_.label_map_[key]=FRONTIER_DIS;
        supported.frtd_.frt_map_[key]=Eigen::Vector3f::UnitZ();points.push_back(p);
      }
      return points;
    };
    auto ghost=layer(1.2f); // Inside the fixture ROI, unsupported by its floor.
    supported.update_updating_aabb(ghost);supported.cluster_frts(ghost,created,removed);
    assert(!supported.hasInspectionFrontiers());
    assert(supported.unclustered_frts_.size()==ghost.size());
    for(const auto&p:ghost)assert(supported.get_state(p)==FRONTIER_DIS);
    auto measured=layer(0.f);
    supported.update_updating_aabb(measured);supported.cluster_frts(measured,created,removed);
    assert(supported.hasInspectionFrontiers());
    supported.cluster_list_.clear();
    pcl::PointCloud<PointType> newly_measured;newly_measured.points=ghost;
    auto new_msg=make_shared<sensor_msgs::msg::PointCloud2>();pcl::toROSMsg(newly_measured,*new_msg);
    support_lio->updateCloudMapOdometry(new_msg,seed_odom);
    supported.cluster_frts({},created,removed); // No fresh frontier notification.
    assert(supported.hasInspectionFrontiers());
    for(const auto&p:ghost)assert(supported.get_state(p)==FRONTIER_DIS);
    auto seed_graph=make_shared<TopoGraph>();FrontierManager seeded;
    seeded.init(manager.nh_,seed_lio,seed_graph);seeded.frtp_.floor_range_frontiers_=true;
    seeded.frtp_.good_observation_trust_length_=8.;seeded.frtp_.good_observation_force_trust_length_=4.;
    vector<ClusterInfo::Ptr> seed_clusters;vector<int> seed_removed;
    seeded.updateFrontierClusters(seed_clusters,seed_removed);
    int seeds=0;for(const auto&entry:seeded.frtd_.label_map_)if(entry.second==FRONTIER_DIS)++seeds;
    assert(seeds>0);
    // Excluded floor labels alone must not prolong a wall/shelf mission.
    FrontierManager floor_only;
    PointType floor;floor.x=0.f;floor.y=0.f;floor.z=0.f;
    ByteArrayRaw floor_key;manager.pos2bytes(floor,floor_key);
    floor_only.frtd_.label_map_[floor_key]=FRONTIER_DIS;
    floor_only.frtd_.frt_map_[floor_key]=Eigen::Vector3f::UnitZ();
    assert(!floor_only.hasInspectionFrontiers());
    std::cout<<"PASS: sparse arrivals accumulate, duplicates removed, unassigned cells retry, completion uses actionable native frontier clusters\n";
  }
};
int main(int argc,char**argv) {
  epic_ros::init(argc,argv,"frontier_clustering_regression");epic_ros::NodeHandle nh;
  auto lio=std::make_shared<LIOInterface>();lio->init(nh);
  auto graph=std::make_shared<TopoGraph>();FrontierManager manager;manager.init(nh,lio,graph);
  FrontierClusteringRegression::run(manager);rclcpp::shutdown();
}
