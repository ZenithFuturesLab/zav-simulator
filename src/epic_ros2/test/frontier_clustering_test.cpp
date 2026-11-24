#include <frontier_manager/frontier_manager.h>
#include <cassert>
#include <iostream>

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
