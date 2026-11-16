#include <path_search/astar.h>
#include <path_search/segment_clearance.hpp>
#include <ros_interface/ros2/ros2_interface.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
namespace path_search {
class AstarClearanceRegression {
 static void require(bool b,const char*m){if(!b)throw std::runtime_error(m);}
 public:
 static void run(){
  auto file=std::filesystem::temp_directory_path()/"zav_astar_body_clearance_test.yaml";
  std::ofstream f(file);
  f<<"astar: {map_voxel_num: [100, 80, 40], allow_diag: true, heu_type: 2, visual_process: false, debug_visualization_en: false}\n"
     "rog_map:\n  resolution: 0.1\n  inflation_resolution: 0.1\n  inflation_step: 2\n  map_size: [8, 8, 6]\n"
     "  fix_map_origin: [0, 0, 1.5]\n  virtual_ground_height: -1\n  virtual_ceil_height: 4\n  map_sliding: {enable: false}\n"
     "  ros_callback: {enable: false}\n  visualization: {enable: false}\n  load_pcd_en: false\n  intensity_thresh: -10\n  point_filt_num: 1\n"
     "  raycasting: {enable: false, local_update_box: [8, 8, 6], ray_range: [0.5, 12], p_min: 0.12, p_miss: 0.49, p_free: 0.499, p_occ: 0.85, p_hit: 0.9, p_max: 0.98}\n";
  f.close();auto node=std::make_shared<rclcpp::Node>("astar_body_clearance_regression");
  auto map=std::make_shared<rog_map::ROGMapROS>(node,file.string());auto ros=std::make_shared<ros_interface::Ros2Interface>(node);
  rog_map::PointCloud cloud;rog_map::PclPoint obstacle;obstacle.x=.05;obstacle.y=.05;obstacle.z=1.55;obstacle.intensity=1.;cloud.push_back(obstacle);
  auto update=[&]{map->updateProbMap(cloud,std::make_pair(Eigen::Vector3d(-1,0,1.5),Eigen::Quaterniond::Identity()));};update();
  Astar astar(file.string(),ros,map);astar.setBodyClearanceRadius(.43);
  const Vec3f start(-1.05,.05,1.55),goal(1.55,.05,1.55);
  const int flag=ON_PROB_MAP|UNKNOWN_AS_FREE|USE_INF_NEIGHBOR;
  require(astar.setup(start,goal,flag,4.)==SUCCESS,"setup failed");
  Vec3i id;
  astar.posToGlobalIndex(Vec3f(.05,.05,1.55),id);
  require(astar.neighborHaveOne(rog_map::OCCUPIED,id),"isolated center occupied cell escaped radius check");
  astar.posToGlobalIndex(Vec3f(.45,.15,1.55),id);
  require(astar.neighborHaveOne(rog_map::OCCUPIED,id),"0.412m candidate must be blocked for R0.43");
  require(!segmentHasBodyClearance(map,Vec3f(.45,.15,1.55),Vec3f(.45,.25,1.55),.43),"previously rejected CIRI seed incorrectly passed");
  astar.posToGlobalIndex(Vec3f(.65,.05,1.55),id);
  require(!astar.neighborHaveOne(rog_map::OCCUPIED,id),"clear 0.6m point incorrectly blocked");
  vec_Vec3f path;auto result=astar.pointToPointPathSearch(start,goal,flag,4.,path);
  require(result==REACH_GOAL&&path.size()>2,"body-aware search must detour around the obstacle");
  for(size_t i=1;i<path.size();++i)require(segmentHasBodyClearance(map,path[i-1],path[i],.43),"A* edge violates CIRI radius");
  // Updating the map and starting another search must invalidate cached clearance.
  obstacle.x=.65;cloud.clear();cloud.push_back(obstacle);update();
  require(astar.setup(start,goal,flag,4.)==SUCCESS,"second setup failed");
  astar.posToGlobalIndex(Vec3f(.65,.05,1.55),id);
  require(astar.neighborHaveOne(rog_map::OCCUPIED,id),"new obstacle was hidden by previous-search cache");
  std::filesystem::remove(file);
  std::cout<<"0.2m display inflation + R0.43 body-aware A*: detour, edge clearance, occupied center, map update passed\n";
 }
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);try{path_search::AstarClearanceRegression::run();}catch(const std::exception&e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
