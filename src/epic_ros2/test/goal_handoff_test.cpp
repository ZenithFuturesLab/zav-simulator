#include <epic_planner/frontier_route.h>
#include <rcl/time.h>
#include <epic_planner/fast_exploration_manager.h>
#include <epic_planner/expl_data.h>
#include <plan_manage/planner_manager.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cassert>
#include <gcopter/geo_utils.hpp>
#include <iostream>

int main(int argc,char** argv) {
  {
    using namespace fast_planner;
    const Eigen::Vector3f heading(0,1,0);
    const FrontierRoute forward={{0,0,1.7},{0,8,1.7}},right={{0,0,1.7},{8,0,1.7}},
        left={{0,0,1.7},{-8,0,1.7}},back={{0,0,1.7},{0,-8,1.7}};
    assert(frontierContinuationCost(forward,heading)<frontierContinuationCost(right,heading));
    assert(frontierContinuationCost(right,heading)<frontierContinuationCost(left,heading));
    assert(frontierContinuationCost(left,heading)<frontierContinuationCost(back,heading));
    FrontierRoute aisle={{0,0,1.7},{.8,3,1.7},{-.8,6,1.7},{.8,9,1.7},{0,12,1.7}};
    const auto free=[](const auto&,const auto&){return true;};
    assert(simplifyFrontierRoute(aisle,1.5f,free).size()==2);
    assert(simplifyFrontierRoute(aisle,1.5f,[](const auto&,const auto&){return false;}).size()==aisle.size());
    const FrontierRoute reversal={{0,0,1.7},{8,0,1.7},{0,0,1.7},{0,10,1.7}};
    assert(simplifyFrontierRoute(reversal,1.5f,free).size()==reversal.size());
    double progress=0.;auto suffix=remainingFrontierRoute(reversal,{1,0,1.7},{0,0,1.7},progress);
    assert(!suffix.empty() && progress>.9 && progress<1.1);
    suffix=remainingFrontierRoute(reversal,{.2,0,1.7},{1,0,1.7},progress);
    assert(!suffix.empty() && progress>=1. && progress<4.);
    std::cout<<"PASS: forward/right/left/reverse frontier priority, checked straight aisle shortcut, no crossing-progress jump\n";
  }
  epic_ros::init(argc,argv,"goal_handoff_regression");
  epic_ros::NodeHandle nh;
  auto lio=std::make_shared<LIOInterface>();lio->init(nh);
  lio->lp_->global_map_min_boundary_=Eigen::Vector3f(-20,-20,-5);
  lio->lp_->global_map_max_boundary_=Eigen::Vector3f(20,20,5);
  lio->ld_->first_map_flag_=true;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for(float x=-8;x<=8;x+=.5f)for(float y=-4;y<=4;y+=.5f)cloud.emplace_back(x,y,0);
  auto scan=[&]() {
    auto msg=std::make_shared<sensor_msgs::msg::PointCloud2>();pcl::toROSMsg(cloud,*msg);
    auto odom=std::make_shared<nav_msgs::msg::Odometry>();odom->pose.pose.position.z=2;odom->pose.pose.orientation.w=1;
    lio->updateCloudMapOdometry(msg,odom);
  };
  scan();
  auto graph=std::make_shared<TopoGraph>();graph->min_bd=lio->lp_->global_map_min_boundary_;
  graph->bubble_min_radius_=.9;
  graph->init_region_size_x_=graph->init_region_size_y_=graph->init_region_size_z_=5.;
  graph->parallel_bubble_astar_=std::make_shared<ParallelBubbleAstar>();graph->parallel_bubble_astar_->init(nh,lio);
  graph->lidar_map_interface_=lio;
  for(int x=0;x<8;++x)for(int y=0;y<8;++y)for(int z=0;z<2;++z) {
    Eigen::Vector3i id(x,y,z);graph->reg_map_idx2ptr_[id]=std::make_shared<RegionNode>(id);
  }
  auto start=std::make_shared<TopoNode>();start->center_=Eigen::Vector3f(-2,0,2);
  Eigen::Vector3i si;graph->getIndex(start->center_,si);graph->getRegionNode(si)->topo_nodes_.insert(start);
  graph->odom_node_=start;
  
  auto goal=std::make_shared<TopoNode>();goal->center_=Eigen::Vector3f(-1,0,2);
  Eigen::Vector3i old;graph->getIndex(goal->center_,old);graph->getRegionNode(old)->topo_nodes_.insert(goal);
  goal->neighbors_.insert(start);start->neighbors_.insert(goal);
  goal->paths_[start]={goal->center_,start->center_};start->paths_[goal]={start->center_,goal->center_};
  goal->weight_[start]=1.;start->weight_[goal]=1.;
  FastExplorationManager manager;manager.ed_=std::make_shared<ExplorationData>();
  manager.planner_manager_=std::make_shared<FastPlannerManager>();manager.planner_manager_->topo_graph_=graph;
  manager.planner_manager_->lidar_map_interface_=lio;
  manager.ed_->next_goal_node_=goal;manager.ed_->global_tour_={start->center_,Eigen::Vector3f(2,0,2)};
  manager.updateGoalNode();
  Eigen::Vector3i next;graph->getIndex(goal->center_,next);
  assert((next-old).norm()>0);assert(!graph->getRegionNode(old)->topo_nodes_.count(goal));
  assert(graph->getRegionNode(next)->topo_nodes_.count(goal));assert(goal->neighbors_.count(start));
  assert((goal->paths_.at(start).front()-goal->center_).norm()<1e-5);
  assert((start->paths_.at(goal).back()-goal->center_).norm()<1e-5);
  
  // A new selected goal becomes occupied. It must have no stale edge/path to
  // the old target; the normal failure/landing policy handles no connection.
  // Native updateOdomNode deliberately keeps only outgoing odom edges.
  // Moving a goal must also detach this incoming one-way edge.
  goal->neighbors_.erase(start);goal->paths_.erase(start);goal->weight_.erase(start);
  cloud.emplace_back(2,2,2);scan();manager.ed_->global_tour_[1]=Eigen::Vector3f(2,2,2);
  manager.updateGoalNode();assert(goal->neighbors_.empty());assert(goal->paths_.empty());
  assert(!start->neighbors_.count(goal));assert(!start->paths_.count(goal));
  // Clearing the target selection recovers normal connections on the same node.
  manager.ed_->global_tour_[1]=Eigen::Vector3f(2,0,2);manager.updateGoalNode();
  assert(goal->neighbors_.count(start));assert((goal->paths_.at(start).front()-goal->center_).norm()<1e-5);
  // A graph edge can remain collision-free while its cached endpoint belongs
  // to an older goal. The local route must reach the CURRENT selected goal.
  fast_planner::FastSearcher localSearcher;localSearcher.init(graph,nullptr);
  const auto healthyEdge=start->paths_.at(goal);
  start->paths_[goal]={start->center_,Eigen::Vector3f(-1,0,2),start->center_};
  vector<Eigen::Vector3f> repairedGuide={Eigen::Vector3f(99,99,99)};
  assert(localSearcher.search(start,Eigen::Vector3f::Zero(),goal,.05,repairedGuide)==fast_planner::BubbleAstar::REACH_END);
  assert((repairedGuide.front()-start->center_).norm()<1e-5);
  assert((repairedGuide.back()-goal->center_).norm()<1e-5);
  assert((goal->center_-Eigen::Vector3f(2,0,2)).norm()<1e-5);
  start->paths_[goal]=healthyEdge;
  // Updated map occupation in the MIDDLE of a cached edge must trigger
  // native replanning, even when both cached endpoints still match.
  start->paths_[goal]={start->center_,Eigen::Vector3f(2,2,2),goal->center_};
  repairedGuide.clear();
  assert(localSearcher.search(start,Eigen::Vector3f::Zero(),goal,.2,repairedGuide)==fast_planner::BubbleAstar::REACH_END);
  for(const auto &point:repairedGuide)assert(lio->getDisToOcc(point)>.5);
  assert((repairedGuide.back()-goal->center_).norm()<1.e-5);
  start->paths_[goal]=healthyEdge;
  // A shortening check can skip an unsafe original segment. Validate the
  // geometry that is actually returned, not a different shortened copy.
  start->paths_[goal]={start->center_,Eigen::Vector3f(1,2,2),
      Eigen::Vector3f(3,2,2),Eigen::Vector3f(0,0,2),goal->center_};
  repairedGuide.clear();
  assert(localSearcher.search(start,Eigen::Vector3f::Zero(),goal,.2,repairedGuide)==fast_planner::BubbleAstar::REACH_END);
  for(size_t j=1;j<repairedGuide.size();++j) {
    vector<Eigen::Vector3f> segment={repairedGuide[j-1],repairedGuide[j]};
    assert(graph->parallel_bubble_astar_->collisionCheck_shortenPath(segment));
  }
  start->paths_[goal]=healthyEdge;
  // The live odom center advances between graph updates. Preserve its cached
  // departure prefix for the existing MINCO-boundary reanchor, rather than
  // misclassifying normal odom latency as a stale selected goal.
  const auto savedStart=start->center_;
  const Eigen::Vector3f cachedBend(-2,1,2);
  start->paths_[goal]={savedStart,cachedBend,goal->center_};
  start->center_.x()+=.5f;repairedGuide.clear();
  assert(localSearcher.search(start,Eigen::Vector3f::Zero(),goal,.05,repairedGuide)==fast_planner::BubbleAstar::REACH_END);
  assert(repairedGuide.size()==3 && (repairedGuide[1]-cachedBend).norm()<1e-5);
  start->center_=savedStart;start->paths_[goal]=healthyEdge;
  std::cout<<"PASS: stale collision-free local edge repaired to current selected goal, previous output cleared\n";
  {
    const Eigen::Vector3d forward(2,0,0);
    const vector<Eigen::Vector3f> routeForward={Eigen::Vector3f(0,0,2),Eigen::Vector3f(3,0,2),Eigen::Vector3f(-4,0,2)};
    const vector<Eigen::Vector3f> routeBackward={Eigen::Vector3f(0,0,2),Eigen::Vector3f(-3,0,2),Eigen::Vector3f(4,0,2)};
    assert(FastExplorationManager::initialDirectionCost(routeForward,forward,5.,1.2)<1.e-6);
    assert(std::abs(FastExplorationManager::initialDirectionCost(routeBackward,forward,5.,1.2)-5.*M_PI/1.2)<1.e-5);
    assert(FastExplorationManager::initialDirectionCost(routeBackward,Eigen::Vector3d::Zero(),5.,1.2)==0.);
    assert(FastExplorationManager::initialDirectionCost(routeBackward,forward,0.,1.2)==0.);
    assert(FastExplorationManager::initialDirectionCost(routeBackward,forward,5.,0.)==0.);
    const vector<Eigen::Vector3f> corner={Eigen::Vector3f(0,0,2),Eigen::Vector3f(0,4,2)};
    assert(std::abs(FastExplorationManager::initialDirectionCost(corner,forward,5.,1.2)-5.*M_PI/2./1.2)<1.e-5);
    std::cout<<"PASS: optional initial-route momentum cost uses real route direction, stationary/default costs preserved\n";
  }
  // Optional LOCAL lookahead preserves global visiting order/goal identity.
  FastExplorationManager look;look.ed_=std::make_shared<ExplorationData>();
  look.planner_manager_=manager.planner_manager_;look.planner_manager_->local_data_.curr_vel_.setZero();
  look.ed_->next_goal_node_=std::make_shared<TopoNode>();look.ed_->next_goal_node_->center_=Eigen::Vector3f(-1,0,2);
  look.ed_->global_tour_={Eigen::Vector3f(-2,0,2),Eigen::Vector3f(-1,0,2),Eigen::Vector3f(1,0,2),Eigen::Vector3f(4,0,2)};
  vector<Eigen::Vector3f> localGuide={look.ed_->global_tour_[0],look.ed_->global_tour_[1]};
  const auto savedTour=look.ed_->global_tour_;const auto savedGoal=look.ed_->next_goal_node_->center_;
  vector<Eigen::Vector3f> lagging={Eigen::Vector3f(-2,0,2),Eigen::Vector3f(-1,0,2),Eigen::Vector3f(1,0,2),Eigen::Vector3f(4,0,2)};
  const Eigen::Vector3f movingBoundary(-.5f,0,2);
  assert(look.reanchorLocalPath(lagging,movingBoundary,Eigen::Vector3f(1,0,0)));
  assert((lagging.front()-movingBoundary).norm()<1.e-6);
  assert(lagging[1].x()>movingBoundary.x()); // No artificial backward odom leg.
  vector<Eigen::Vector3f> loopedCached={Eigen::Vector3f(-2,0,2),Eigen::Vector3f(1,0,2),
      Eigen::Vector3f(-1,0,2),Eigen::Vector3f(1.5f,0,2),Eigen::Vector3f(4,0,2)};
  assert(look.reanchorLocalPath(loopedCached,movingBoundary,Eigen::Vector3f(1,0,0)));
  assert((loopedCached.back()-Eigen::Vector3f(4,0,2)).norm()<1.e-5);
  for(size_t i=1;i<loopedCached.size();++i)
    assert(loopedCached[i].x()>=loopedCached[i-1].x()-1.e-5); // Discard old backward loop.
  vector<Eigen::Vector3f> sparseCached={Eigen::Vector3f(-6,0,2),Eigen::Vector3f(4,0,2)};
  assert(look.reanchorLocalPath(sparseCached,movingBoundary,Eigen::Vector3f(1,0,0)));
  assert((sparseCached.front()-movingBoundary).norm()<1.e-5);
  assert((sparseCached.back()-Eigen::Vector3f(4,0,2)).norm()<1.e-5);
  for(const auto&p:sparseCached)assert(p.x()>=movingBoundary.x()-1.e-5);
  assert((look.ed_->next_goal_node_->center_-savedGoal).norm()<1.e-5);
  for(size_t i=0;i<savedTour.size();++i)assert((look.ed_->global_tour_[i]-savedTour[i]).norm()<1.e-5);
  {
    look.ed_->global_tour_={Eigen::Vector3f(-2,0,2),Eigen::Vector3f(0,0,2),Eigen::Vector3f(0,3,2)};
    const auto cornerTour=look.ed_->global_tour_;
    localGuide={cornerTour[0],cornerTour[1]};assert(!look.extendTourPath(localGuide));
    nh.native()->declare_parameter<double>("exploration.tour_lookahead_distance",0.);
    nh.native()->set_parameter(rclcpp::Parameter("exploration.tour_lookahead_distance",10.));
    assert(look.extendTourPath(localGuide));assert((localGuide.back()-cornerTour[2]).norm()<1.e-5);
    for(size_t i=0;i<cornerTour.size();++i)assert((cornerTour[i]-look.ed_->global_tour_[i]).norm()<1.e-5);
    assert((look.ed_->next_goal_node_->center_-savedGoal).norm()<1.e-5);
    look.ed_->global_tour_={Eigen::Vector3f(-2,0,2),Eigen::Vector3f(0,0,2),Eigen::Vector3f(-3,0,2)};
    localGuide={look.ed_->global_tour_[0],look.ed_->global_tour_[1]};assert(look.extendTourPath(localGuide));
    assert((localGuide.back()-look.ed_->global_tour_[2]).norm()<1.e-5);
    look.directional_frontiers_=true;
    localGuide={look.ed_->global_tour_[0],look.ed_->global_tour_[1]};
    assert(!look.extendTourPath(localGuide)); // A rear frontier is deferred, not visited.
    assert(localGuide.size()==2 && look.ed_->global_tour_.size()==3);
    look.ed_->global_tour_.push_back(Eigen::Vector3f(3,0,2));
    assert(look.extendTourPath(localGuide));
    assert((localGuide.back()-Eigen::Vector3f(3,0,2)).norm()<1.e-5);
    assert((look.ed_->global_tour_[2]-Eigen::Vector3f(-3,0,2)).norm()<1.e-5);
    look.directional_frontiers_=false;
    // The optional guide is arc-bounded even when a future native leg is long.
    look.ed_->global_tour_={Eigen::Vector3f(-2,0,2),Eigen::Vector3f(0,0,2),Eigen::Vector3f(0,8,2)};
    nh.native()->set_parameter(rclcpp::Parameter("exploration.tour_lookahead_distance",5.));
    localGuide={look.ed_->global_tour_[0],look.ed_->global_tour_[1]};assert(look.extendTourPath(localGuide));
    double arc=0.;for(size_t i=1;i<localGuide.size();++i)arc+=(localGuide[i]-localGuide[i-1]).norm();assert(std::abs(arc-5.)<1.e-4);
    nh.native()->set_parameter(rclcpp::Parameter("exploration.tour_lookahead_distance",0.));
    look.ed_->global_tour_=savedTour;
    std::cout<<"PASS: native TSP corner/return-leg preview, unchanged global order and bounded certified guide\n";
  }
  std::cout<<"PASS: local straight-prefix lookahead, global tour/goal unchanged, angular/order/velocity guards\n";
  
  {
    // Exercise native historical reconnects after revisiting an older pose.
    // The helper searches history->odom, but stores an odom->history edge.
    auto historyGraph=std::make_shared<TopoGraph>();
    auto historySearcher=graph->parallel_bubble_astar_;
    historyGraph->init(nh,lio,historySearcher);
    historyGraph->reg_map_idx2ptr_.clear();
    historyGraph->min_bd=graph->min_bd;
    historyGraph->init_region_size_x_=historyGraph->init_region_size_y_=historyGraph->init_region_size_z_=5.;
    historyGraph->lidar_map_interface_=lio;historyGraph->parallel_bubble_astar_=graph->parallel_bubble_astar_;
    for(const auto &entry:graph->reg_map_idx2ptr_)
      historyGraph->reg_map_idx2ptr_[entry.first]=std::make_shared<RegionNode>(entry.first);
    historyGraph->odom_node_=std::make_shared<TopoNode>();
    historyGraph->odom_node_->center_=Eigen::Vector3f(-2,0,2);
    historyGraph->updateHistoricalOdoms();assert(historyGraph->history_odom_nodes_.size()==1);
    historyGraph->odom_node_->center_=Eigen::Vector3f(-10,0,2);
    historyGraph->updateHistoricalOdoms();assert(historyGraph->history_odom_nodes_.size()==2);
    historyGraph->odom_node_->center_=Eigen::Vector3f(-3,0,2);
    historyGraph->updateHistoricalOdoms();
    const auto current=historyGraph->odom_node_,older=historyGraph->history_odom_nodes_.front();
    const auto &edge=current->paths_.at(older);
    assert(edge.size()>=2);
    assert((edge.front()-current->center_).norm()<1.e-5);
    assert((edge.back()-older->center_).norm()<1.e-5);
    const auto &reverse=older->paths_.at(current);
    assert((reverse.front()-older->center_).norm()<1.e-5);
    assert((reverse.back()-current->center_).norm()<1.e-5);
    std::cout<<"PASS: historical reconnect paths retain source-to-target orientation in both directions\n";
  }
  // Build a real point-cloud wall. Node search must enforce the configured
  // 0.89m clearance consistently with native one-shot collision checking.
  for(float y=-4;y<=4;y+=.1f)for(float z=.5f;z<=3.5f;z+=.1f)cloud.emplace_back(0,y,z);
  scan();
  
  std::unordered_set<Eigen::Vector3i,v3i_hash> safe,bad;
  auto probe=std::make_shared<ParallelBubbleAstar::Node>();probe->position=Eigen::Vector3f(-.95f,0,2);
  assert(graph->parallel_bubble_astar_->isNodeSafe(probe,lio->lp_->global_map_min_boundary_,lio->lp_->global_map_max_boundary_,safe,bad));
  safe.clear();bad.clear();probe->position=Eigen::Vector3f(-.85f,0,2);
  assert(!graph->parallel_bubble_astar_->isNodeSafe(probe,lio->lp_->global_map_min_boundary_,lio->lp_->global_map_max_boundary_,safe,bad));
  safe.clear();bad.clear();probe->position=Eigen::Vector3f(-6,0,2);
  const double previousResolution=graph->parallel_bubble_astar_->resolution_;
  graph->parallel_bubble_astar_->resolution_=.1;graph->parallel_bubble_astar_->inv_resolution_=10.;
  assert(graph->parallel_bubble_astar_->isNodeSafe(probe,lio->lp_->global_map_min_boundary_,lio->lp_->global_map_max_boundary_,safe,bad));
  assert(safe.size()>1); // Open space must reuse bounded clearance certificates.
  safe.clear();bad.clear();probe->position=Eigen::Vector3f(-.895f,0,2);
  assert(graph->parallel_bubble_astar_->isNodeSafe(probe,lio->lp_->global_map_min_boundary_,lio->lp_->global_map_max_boundary_,safe,bad));
  Eigen::Vector3i offgrid;graph->parallel_bubble_astar_->posToIndex(probe->position,offgrid);
  graph->parallel_bubble_astar_->IndexToPos(probe->position,offgrid);
  assert(!graph->parallel_bubble_astar_->isNodeSafe(probe,lio->lp_->global_map_min_boundary_,lio->lp_->global_map_max_boundary_,safe,bad));
  const double previousMin=graph->parallel_bubble_astar_->flight_min_z_,previousMax=graph->parallel_bubble_astar_->flight_max_z_;
  graph->parallel_bubble_astar_->flight_min_z_=1.35;graph->parallel_bubble_astar_->flight_max_z_=2.15;
  vector<Eigen::Vector3f> topLayer;
  assert(graph->parallel_bubble_astar_->search(Eigen::Vector3f(-1.2f,0,2),Eigen::Vector3f(-1.2f,1,2.15f),topLayer,.01,false,true)==ParallelBubbleAstar::REACH_END);
  graph->parallel_bubble_astar_->flight_min_z_=previousMin;graph->parallel_bubble_astar_->flight_max_z_=previousMax;
  graph->parallel_bubble_astar_->resolution_=previousResolution;
  graph->parallel_bubble_astar_->inv_resolution_=1./previousResolution;
  std::cout<<"PASS: bounded free-space cache, off-grid unsafe cell not cached, float boundary layer retained\n";
  // A line parallel to a wall stays0.92m away even when the0.03m clearance
  // balls at0.1m grid vertices do not overlap. Keep the original safe edge.
  vector<Eigen::Vector3f> narrow;
  for(int i=0;i<=10;++i)narrow.emplace_back(-.92f,i*.1f,2.f);
  assert(graph->parallel_bubble_astar_->collisionCheck_shortenPath(narrow));
  vector<Eigen::Vector3f> crossing={Eigen::Vector3f(-.92f,0,2),Eigen::Vector3f(.92f,0,2)};
  assert(!graph->parallel_bubble_astar_->collisionCheck_shortenPath(crossing));
  std::cout<<"PASS: certified narrow grid edges retained, real wall crossing rejected at unchanged0.89m clearance\n";
  look.ed_->global_tour_=savedTour;look.planner_manager_->local_data_.curr_vel_.setZero();
  vector<Eigen::Vector3f> blockedAnchor={Eigen::Vector3f(-1.5f,0,2),Eigen::Vector3f(1.5f,0,2)};
  assert(look.reanchorLocalPath(blockedAnchor,Eigen::Vector3f(-1.2f,0,2),Eigen::Vector3f(1,0,0)));
  assert((blockedAnchor.front()-Eigen::Vector3f(-1.2f,0,2)).norm()<1.e-5);
  assert((blockedAnchor.back()-Eigen::Vector3f(1.5f,0,2)).norm()<1.e-5);
  assert(graph->parallel_bubble_astar_->collisionCheck_shortenPath(blockedAnchor));
  // The existing full A* may route around a finite wall, but must reject a
  // goal inside that wall and leave no unchecked cached-prefix command.
  vector<Eigen::Vector3f> occupiedGoal={Eigen::Vector3f(-1.5f,0,2),Eigen::Vector3f(0,0,2)};
  assert(!look.reanchorLocalPath(occupiedGoal,Eigen::Vector3f(-1.2f,0,2),Eigen::Vector3f(1,0,0)));
  assert(occupiedGoal.empty());
  
  auto savedConfiguration=std::move(look.planner_manager_->gcopter_config_);
  look.planner_manager_->gcopter_config_=std::make_unique<GcopterConfig>();
  look.planner_manager_->gcopter_config_->startCorridorRecovery=true;
  look.planner_manager_->gcopter_config_->dilateRadiusHard=.45;
  look.planner_manager_->gcopter_config_->startRecoveryMargin=.05;
  
  vector<Eigen::Vector3f> nearWall={Eigen::Vector3f(-1.2f,0,2),Eigen::Vector3f(-2.f,2,2)};
  const Eigen::Vector3f nearBoundary(-.65f,0,2);
  assert(look.reanchorLocalPath(nearWall,nearBoundary,Eigen::Vector3f(1,0,0)));
  assert((nearWall.front()-nearBoundary).norm()<1.e-5);
  assert((nearWall.back()-Eigen::Vector3f(-2.f,2,2)).norm()<1.e-5);
  for(float t=0;t<=1.f;t+=.02f)
    assert(lio->getDisToOcc(Eigen::Vector3f(nearWall[0]*(1-t)+nearWall[1]*t))>=.5f-1.e-5);
  vector<Eigen::Vector3f> unsafeBoundary={Eigen::Vector3f(-1.2f,0,2),Eigen::Vector3f(-2.f,2,2)};
  assert(!look.reanchorLocalPath(unsafeBoundary,Eigen::Vector3f(-.3f,0,2),Eigen::Vector3f(1,0,0)));
  assert(unsafeBoundary.empty());
  look.planner_manager_->gcopter_config_=std::move(savedConfiguration);
  std::cout<<"PASS: moving boundary removes stale backward prefix only with native clear join; wall shortcut rejected, global order unchanged\n";
  {
    const double oldTimeout=nh.native()->get_parameter("parallel_astar.goal_connection_timeout").as_double();
    nh.native()->set_parameter(rclcpp::Parameter("parallel_astar.goal_connection_timeout",1.e-9));
    FastExplorationManager handoff;handoff.planner_manager_=look.planner_manager_;
    handoff.ed_=std::make_shared<ExplorationData>();handoff.ed_->next_goal_node_=std::make_shared<TopoNode>();
    handoff.ed_->next_goal_node_->center_=Eigen::Vector3f(-2,0,2);
    handoff.ed_->next_goal_node_->is_viewpoint_=true;
    auto selected=std::make_shared<TopoNode>();selected->center_=Eigen::Vector3f(3.5f,0,2);selected->is_viewpoint_=true;
    auto odom=graph->odom_node_;
    vector<Eigen::Vector3f> corners={selected->center_,Eigen::Vector3f(3.5f,5,2),
        Eigen::Vector3f(-2,5,2),odom->center_},detour;
    for(size_t i=0;i+1<corners.size();++i) {
      const int steps=std::ceil((corners[i+1]-corners[i]).norm()/.1f);
      for(int j=0;j<steps;++j)detour.push_back(corners[i]+float(j)/steps*(corners[i+1]-corners[i]));
    }
    detour.push_back(corners.back());
    auto checked=detour;
    if(!graph->parallel_bubble_astar_->collisionCheck_shortenPath(checked)) {
      float minimum=1.e9f;Eigen::Vector3f nearest;
      for(const auto&p:detour)if(lio->getDisToOcc(p)<minimum){minimum=lio->getDisToOcc(p);nearest=p;}
      std::cerr<<"DETOUR fixture clearance "<<minimum<<" at "<<nearest.transpose()
               <<" required "<<graph->parallel_bubble_astar_->safe_distance_<<" odom "<<odom->center_.transpose()<<std::endl;
      assert(false);
    }
    selected->neighbors_.insert(odom);selected->paths_[odom]=detour;
    odom->neighbors_.insert(selected);odom->paths_[selected]=vector<Eigen::Vector3f>(detour.rbegin(),detour.rend());
    Eigen::Vector3i selectedIndex;graph->getIndex(selected->center_,selectedIndex);
    graph->getRegionNode(selectedIndex)->topo_nodes_.insert(selected);
    handoff.ed_->global_tour_={odom->center_,selected->center_};handoff.updateGoalNode(selected);
    vector<TopoNode::Ptr> temporary={selected};graph->removeNodes(temporary);
    const auto goalNode=handoff.ed_->next_goal_node_;
    assert(goalNode->neighbors_.count(odom) && odom->neighbors_.count(goalNode));
    assert((goalNode->paths_.at(odom).front()-goalNode->center_).norm()<1.e-5);
    assert((goalNode->paths_.at(odom).back()-odom->center_).norm()<1.e-5);
    // Also retain a route whose last connection uses TWO temporary viewpoints.
    auto bridge=std::make_shared<TopoNode>();bridge->center_=corners[1];bridge->is_viewpoint_=true;
    selected->neighbors_.clear();selected->paths_.clear();selected->weight_.clear();
    vector<Eigen::Vector3f> firstPart,lastPart;
    for(const auto&p:detour) {
      if((p-corners[1]).norm()<1.e-4 || !lastPart.empty())lastPart.push_back(p);
      else firstPart.push_back(p);
    }
    firstPart.push_back(corners[1]);
    selected->neighbors_.insert(bridge);bridge->neighbors_.insert(selected);
    selected->paths_[bridge]=firstPart;bridge->paths_[selected]=vector<Eigen::Vector3f>(firstPart.rbegin(),firstPart.rend());
    bridge->neighbors_.insert(odom);odom->neighbors_.insert(bridge);
    bridge->paths_[odom]=lastPart;odom->paths_[bridge]=vector<Eigen::Vector3f>(lastPart.rbegin(),lastPart.rend());
    double firstCost,lastCost;graph->parallel_bubble_astar_->calculatePathCost(firstPart,firstCost);
    graph->parallel_bubble_astar_->calculatePathCost(lastPart,lastCost);
    selected->weight_[bridge]=bridge->weight_[selected]=firstCost;
    bridge->weight_[odom]=odom->weight_[bridge]=lastCost;
    graph->getRegionNode(selectedIndex)->topo_nodes_.insert(selected);
    Eigen::Vector3i bridgeIndex;graph->getIndex(bridge->center_,bridgeIndex);
    graph->getRegionNode(bridgeIndex)->topo_nodes_.insert(bridge);
    handoff.updateGoalNode(selected);temporary={selected,bridge};graph->removeNodes(temporary);
    assert(goalNode->neighbors_.count(odom) && odom->neighbors_.count(goalNode));
    handoff.updateGoalNode();assert(goalNode->neighbors_.count(odom)); // Same target retains its safe edge.
    handoff.ed_->global_tour_[1]=Eigen::Vector3f(100,100,2);handoff.updateGoalNode();
    assert(goalNode->neighbors_.empty() && !odom->paths_.count(goalNode)); // Changed target cannot retain it.
    nh.native()->set_parameter(rclcpp::Parameter("parallel_astar.goal_connection_timeout",oldTimeout));
    std::cout<<"PASS: selected safe connections survive transient removal and same-goal refresh; changed goals clear old edges\n";
  }
  {
    // Warehouse failure: close goal, long obstacle detour. Its complete
    // guide must not defeat the local optimization horizon.
    const vector<Eigen::Vector3f> detour={Eigen::Vector3f(0,0,2),Eigen::Vector3f(0,20,2),
        Eigen::Vector3f(4,20,2),Eigen::Vector3f(4,0,2)};
    const auto saved=detour;
    const vector<Eigen::Vector3f> longLine={Eigen::Vector3f(0,0,2),Eigen::Vector3f(20,0,2)};
    assert(FastPlannerManager::guideHorizon(longLine,6.,0.)==6.);
    assert(FastPlannerManager::guideHorizon(longLine,6.,10.)==10.);
    const vector<Eigen::Vector3f> bend={Eigen::Vector3f(0,0,2),Eigen::Vector3f(4,0,2),Eigen::Vector3f(4,12,2)};
    assert(FastPlannerManager::guideHorizon(bend,6.,10.)==6.);
    const vector<Eigen::Vector3f> reverse={Eigen::Vector3f(0,0,2),Eigen::Vector3f(4,0,2),Eigen::Vector3f(2,0,2),Eigen::Vector3f(12,0,2)};
    assert(FastPlannerManager::guideHorizon(reverse,6.,10.)==6.);
    assert(FastPlannerManager::guideHorizon(longLine,6.,30.)==6.);
    const auto boundedLine=FastPlannerManager::boundedGuidePrefix(longLine,FastPlannerManager::guideHorizon(longLine,6.,10.));
    assert(std::abs((boundedLine.back()-boundedLine.front()).norm()-10.)<1.e-6);
    auto prefix=FastPlannerManager::boundedGuidePrefix(detour,12.);
    double length=0.;for(size_t j=1;j<prefix.size();++j)length+=(prefix[j]-prefix[j-1]).norm();
    assert(std::abs(length-12.)<1.e-6);assert((prefix.back()-Eigen::Vector3d(0,12,2)).norm()<1.e-6);
    for(size_t j=0;j<saved.size();++j)assert((saved[j]-detour[j]).norm()==0.);
    prefix=FastPlannerManager::boundedGuidePrefix({Eigen::Vector3f(0,0,2),Eigen::Vector3f(2,0,2)},12.);
    assert(prefix.size()==2 && (prefix.back()-Eigen::Vector3d(2,0,2)).norm()<1.e-6);
    assert(FastPlannerManager::boundedGuidePrefix(detour,0.).empty());
    std::cout<<"PASS: close-goal long detour obeys local arc-length horizon; global guide and short routes preserved\n";
  }
  FastPlannerManager recovery;recovery.lidar_map_interface_=lio;
  auto intervalBox=[](double lo,double hi) {
    Eigen::MatrixX4d p(6,4);p<<1.,0.,0.,-hi, -1.,0.,0.,lo,
      0.,1.,0.,-1., 0.,-1.,0.,-1., 0.,0.,1.,-2.1, 0.,0.,-1.,1.5;
    return p;
  };
  assert(FastPlannerManager::connectedCorridorPrefix({intervalBox(0,2),intervalBox(1,3),intervalBox(2.5,4)})==3);
  assert(FastPlannerManager::connectedCorridorPrefix({intervalBox(0,2),intervalBox(1,3),intervalBox(4,5)})==2);
  assert(FastPlannerManager::connectedCorridorPrefix({intervalBox(0,2),intervalBox(4,5)})==1);
  assert(FastPlannerManager::connectedCorridorPrefix({intervalBox(2,1)})==0);
  std::cout<<"PASS: final corridor pair checked, disconnected tail excluded, infeasible initial region rejected\n";
  {
    // A slower INITIAL guess must preserve the moving boundary, rather than
    // time-scaling a solved polynomial and changing its velocity/acceleration.
    gcopter::GCOPTER_PolytopeSFC solver;
    Eigen::Matrix<double,3,4> initial=Eigen::Matrix<double,3,4>::Zero(),terminal=initial;
    initial.col(0)<<-2.,0.,2.;initial.col(1)<<.4,0.,0.;
    initial.col(2)<<.1,0.,0.;initial.col(3)<<.05,0.,0.;terminal.col(0)<<-1.,0.,2.;
    Eigen::VectorXd limits(9),weights(9),physical(6);
    limits<<2.5,10000.,.65,10.,28.,2.5,5.,.25,.25;
    weights<<110000.,1500000.,11000.,110000.,1100000.,100000.,100000.,100000.,100000.;
    physical<<1.99,9.80665,.7,.8,.01,.0001;
    assert(solver.setup(100.,.6,initial,terminal,{intervalBox(-4.,-.5),intervalBox(-4.,-.5)},
                        INFINITY,.05,8,limits,weights,physical));
    Trajectory<7> slowerSeed;
    assert(std::isfinite(solver.optimize(slowerSeed,1.e-5,1.,5000.,2.)));
    assert((slowerSeed.getPos(0.)-initial.col(0)).norm()<1.e-7);
    assert((slowerSeed.getVel(0.)-initial.col(1)).norm()<1.e-6);
    assert((slowerSeed.getAcc(0.)-initial.col(2)).norm()<1.e-5);
    assert((slowerSeed.getJer(0.)-initial.col(3)).norm()<1.e-4);
    assert((slowerSeed.getPos(slowerSeed.getTotalDuration())-terminal.col(0)).norm()<1.e-5);
    assert(slowerSeed.getVel(slowerSeed.getTotalDuration()).norm()<1.e-5);
    std::cout<<"PASS: slower MINCO initial seed preserves moving PVAJ and terminal state\n";
  }
  recovery.gcopter_config_=std::make_unique<GcopterConfig>();
  recovery.gcopter_config_->dilateRadiusHard=.45;recovery.gcopter_config_->dilateRadiusSoft=.8;
  recovery.gcopter_config_->flightMinZ=1.35;recovery.gcopter_config_->flightMaxZ=2.15;
  auto box=[]() {Eigen::MatrixX4d poly(6,4);poly<<1.,0.,0.,1.5, -1.,0.,0.,-3.,
      0.,1.,0.,-1., 0.,-1.,0.,-1., 0.,0.,1.,-2.1, 0.,0.,-1.,1.5;return poly;};
  vector<Eigen::MatrixX4d> corridors={box()};
  const Eigen::Vector3d boundary(-1.2,0,2);
  const vector<Eigen::Vector3d> recoveryGuide={boundary,Eigen::Vector3d(-1.8,0,2)};
  assert(!recovery.recoverStartCorridor(boundary,recoveryGuide,corridors));
  recovery.gcopter_config_->startCorridorRecovery=true;
  assert(recovery.recoverStartCorridor(boundary,recoveryGuide,corridors));assert(corridors.size()>=2);
  Eigen::Vector4d boundaryH;boundaryH<<boundary,1.;assert((corridors.front()*boundaryH).maxCoeff()<0.);
  for(size_t i=0;i<corridors.size();++i) {
    Eigen::Matrix3Xd vertices;assert(geo_utils::enumerateVs(corridors[i],vertices));
    for(int j=0;j<vertices.cols();++j) {const Eigen::Vector3d vertex=vertices.col(j);assert(lio->getDisToOcc(vertex)>=.45-1.e-5);}
    if(i)assert(geo_utils::overlap(corridors[i-1],corridors[i],.01));
  }
  corridors={box()};assert(!recovery.recoverStartCorridor(Eigen::Vector3d(-.3,0,2),recoveryGuide,corridors));
  assert(corridors.size()==1 && corridors[0].isApprox(box()));
  assert(!recovery.recoverStartCorridor(Eigen::Vector3d(-2,0,2),recoveryGuide,corridors));
  {
    // Start is already inside a short cell, but its next cell is separated.
    // Holding in the first cell must not prevent a certified native-guide join.
    auto first=box(),next=box();first(1,3)=-1.9; // x in[-1.9,-1.5].
    next(0,3)=2.3;next(1,3)=-3.; // x in[-3,-2.3].
    vector<Eigen::MatrixX4d> split={first,next};
    const Eigen::Vector3d start(-1.7,0,2);
    const vector<Eigen::Vector3d> guide={start,Eigen::Vector3d(-2.7,0,2)};
    assert(FastPlannerManager::connectedCorridorPrefix(split)==1);
    assert(!recovery.recoverStartCorridor(start,guide,split)); // Original start policy.
    assert(recovery.recoverStartCorridor(start,guide,split,1));
    assert(FastPlannerManager::connectedCorridorPrefix(split)==split.size());
    Eigen::Vector4d sh;sh<<start,1.;assert((split.front()*sh).maxCoeff()<0.);
    for(const auto &poly:split) {
      Eigen::Matrix3Xd vertices;assert(geo_utils::enumerateVs(poly,vertices));
      for(int j=0;j<vertices.cols();++j)
        assert(lio->getDisToOcc(Eigen::Vector3d(vertices.col(j)))>=.45-1.e-5);
    }
    // An occupied crossing cannot be patched merely because its tail exists.
    auto across=box();across(0,3)=-2.;across(1,3)=1.;
    split={first,across};const auto saved=split;
    assert(!recovery.recoverStartCorridor(start,{start,Eigen::Vector3d(1.5,0,2)},split,1));
    assert(split.size()==saved.size() && split[0].isApprox(saved[0]) && split[1].isApprox(saved[1]));
    std::cout<<"PASS: disconnected departure rejoins native guide through certified cubes; occupied crossing rejected\n";
  }
  // A valid accepted boundary can be outside the preferred0.6m corridor
  // while retaining0.575m physical clearance, above the hard0.45m guard.
  recovery.gcopter_config_->dilateRadiusSoft=.6;
  recovery.gcopter_config_->startRecoveryMargin=.05;
  corridors={box()};
  const Eigen::Vector3d tightBoundary(-.575,0,2);
  const std::vector<Eigen::Vector3d> tightGuide={tightBoundary,Eigen::Vector3d(-1.8,0,2)};
  assert(recovery.recoverStartCorridor(tightBoundary,tightGuide,corridors));
  for(size_t k=0;k<corridors.size();++k) {
    Eigen::Matrix3Xd vertices;assert(geo_utils::enumerateVs(corridors[k],vertices));
    for(int j=0;j<vertices.cols();++j)assert(lio->getDisToOcc(Eigen::Vector3d(vertices.col(j)))>=.45-1.e-5);
    if(k)assert(geo_utils::overlap(corridors[k-1],corridors[k],.01));
  }
  recovery.gcopter_config_->dilateRadiusSoft=.8;
  recovery.gcopter_config_->startRecoveryMargin=.1;
  std::cout<<"PASS: bounded start connector contains boundary, overlaps FIRI, preserves collision clearance; unsafe start rejected\n";
  // A finite stopping trajectory continues as an endpoint hold, but a genuine
  // collision or nonzero terminal derivatives must never bypass braking checks.
  Piece<7>::CoefficientMat hold=Piece<7>::CoefficientMat::Zero();hold(0,7)=-1.2;hold(2,7)=2.;
  recovery.local_data_.minco_traj_=Trajectory<7>({1.},{hold});recovery.local_data_.duration_=1.;
  assert(!recovery.hasSafeTerminalHold());
  recovery.gcopter_config_->safeTerminalHoldFallback=true;assert(recovery.hasSafeTerminalHold());
  hold(0,6)=.2;recovery.local_data_.minco_traj_=Trajectory<7>({1.},{hold});assert(!recovery.hasSafeTerminalHold());
  hold.setZero();hold(0,7)=-1.9;hold(0,6)=-.2;hold(0,5)=.1;hold(2,7)=2.;
  recovery.local_data_.minco_traj_=Trajectory<7>({1.},{hold});assert(!recovery.hasSafeTerminalHold());
  hold.setZero();hold(0,7)=-.3;hold(2,7)=2.;
  recovery.local_data_.minco_traj_=Trajectory<7>({1.},{hold});assert(!recovery.hasSafeTerminalHold());
  hold.setZero();hold.row(0)<<150.,-600.,900.,-600.,150.,0.,0.,-1.2;hold(2,7)=2.;
  recovery.local_data_.minco_traj_=Trajectory<7>({1.},{hold});assert(recovery.hasSafeTerminalHold());
  double collisionHorizon=0.;const bool wholeSafe=recovery.checkTrajCollision(collisionHorizon,0.);
  assert(!wholeSafe);assert(!(wholeSafe && recovery.hasSafeTerminalHold()));
  std::cout<<"PASS: terminal stop/hold eligibility, nonzero derivatives/occupied endpoint reject, real collision cannot use hold fallback\n";
  // A collision beyond the first second is still a real collision. The
  // strict exploration profile must not repeatedly publish this unsafe tail.
  Eigen::Matrix<double,3,8> lateCollision=Eigen::Matrix<double,3,8>::Zero();
  lateCollision(0,7)=-3.;lateCollision(0,6)=.5;lateCollision(2,7)=2.;
  recovery.local_data_.minco_traj_=Trajectory<7>({8.},{lateCollision});
  recovery.local_data_.duration_=8.;collisionHorizon=0.;
  assert(!recovery.checkTrajCollision(collisionHorizon,0.));assert(collisionHorizon>1.);
  std::cout<<"PASS: full candidate collision check detects unsafe tail beyond first second\n";
  {
    // Clipping near workspace bounds forces the native gap branch. Every
    // cell must apply the same erosion, even if the clipped cell becomes
    // infeasible and must subsequently be rejected by corridor validation.
    const std::vector<Eigen::Vector3d> obstacles={Eigen::Vector3d(0,2,1),
        Eigen::Vector3d(2,0,1),Eigen::Vector3d(0,-2,3),Eigen::Vector3d(-2,0,3)};
    const std::vector<Eigen::Vector3d> guide={Eigen::Vector3d(-2.8,-2.8,.2),Eigen::Vector3d(2.8,2.8,3.8)};
    std::vector<Eigen::MatrixX4d> cover;
    std::unique_ptr<Visualizer> unusedVisualizer;
    sfc_gen::convexCover(unusedVisualizer,guide,obstacles,Eigen::Vector3d(-3,-3,0),
        Eigen::Vector3d(3,3,4),20.,3.,cover,1.e-6,.6);
    assert(cover.size()==3); // Ordered a/midpoint/b fallback, no backward gap or duplicated b cell.
    for(const auto &poly:cover)for(const auto &point:obstacles) {
      assert(poly.allFinite());
      const Eigen::ArrayXd norms=poly.leftCols<3>().rowwise().norm();
      assert(norms.minCoeff()>1.e-9);
      const double margin=((poly.leftCols<3>()*point+poly.col(3)).array()/norms).maxCoeff();
      assert(margin>=.6-1.e-5);
    }
    for(const auto &poly:cover)for(int row=0;row<poly.rows();++row) {
      Eigen::Vector4d plane=poly.row(row).transpose()/poly.row(row).head<3>().norm();
      for(int axis=0;axis<3;++axis) {
        const Eigen::Vector3d normal=Eigen::Vector3d::Unit(axis);
        const double lo=axis==2 ? 0. : -3.,hi=axis==2 ? 4. : 3.;
        if((plane.head<3>()-normal).norm()<1.e-8)assert(plane[3]>=-hi+.6-1.e-5);
        if((plane.head<3>()+normal).norm()<1.e-8)assert(plane[3]>=lo+.6-1.e-5);
      }
    }
    std::cout<<"PASS: every FIRI corridor including gap cells has configured obstacle clearance\n";
  }
  FrontierManager frontier;frontier.init(nh,lio,graph);
  std::cout<<"PASS: moved goal region, path endpoint identity, failed reconnect clears stale edges, recovery\n";
  // Endpoint-only checks miss a polynomial that rises over a rack and returns.
  FastPlannerManager height;
  height.gcopter_config_=std::make_unique<GcopterConfig>();
  height.gcopter_config_->flightMinZ=1.35;height.gcopter_config_->flightMaxZ=2.15;
  height.gcopter_config_->heightPlanningMargin=.02;
  Eigen::MatrixX4d tallBox(6,4);
  tallBox<<1.,0.,0.,-2., -1.,0.,0.,-2., 0.,1.,0.,-2.,
      0.,-1.,0.,-2., 0.,0.,1.,-4., 0.,0.,-1.,0.;
  std::vector<Eigen::MatrixX4d> buffered={tallBox};
  height.addFlightHeightConstraints(buffered);
  Eigen::Matrix3Xd bufferedVertices;assert(geo_utils::enumerateVs(buffered.front(),bufferedVertices));
  assert(bufferedVertices.row(2).minCoeff()>=1.37-1.e-6);
  assert(bufferedVertices.row(2).maxCoeff()<=2.13+1.e-6);
  Eigen::Vector4d raisedGoal;raisedGoal<<0.,0.,2.13-1.e-6,1.;
  assert((buffered.front()*raisedGoal).maxCoeff()<=0.);
  assert(height.gcopter_config_->flightMinZ==1.35 && height.gcopter_config_->flightMaxZ==2.15);
  Piece<7>::CoefficientMat guardOnly=Piece<7>::CoefficientMat::Zero();guardOnly(2,7)=1.36;
  height.local_data_.minco_traj_=Trajectory<7>({1.},{guardOnly});
  assert(height.checkTrajHeight()); // Inner optimization buffer did not change hard acceptance.
  std::cout<<"PASS: buffered planning polytope, raised goal preserved, original hard height guard unchanged\n";
  Piece<7>::CoefficientMat coefficients=Piece<7>::CoefficientMat::Zero();
  coefficients(2,7)=1.7;
  height.local_data_.minco_traj_=Trajectory<7>({1.},{coefficients});assert(height.checkTrajHeight());
  coefficients(2,5)=-4.;coefficients(2,6)=4.;
  height.local_data_.minco_traj_=Trajectory<7>({1.},{coefficients});assert(!height.checkTrajHeight());
  coefficients(2,5)=4.;coefficients(2,6)=-4.;
  height.local_data_.minco_traj_=Trajectory<7>({1.},{coefficients});assert(!height.checkTrajHeight());
  coefficients.setZero();coefficients(2,7)=2.;coefficients(2,6)=120.;coefficients(2,5)=-12000.;
  height.local_data_.minco_traj_=Trajectory<7>({.01},{coefficients});assert(!height.checkTrajHeight());
  coefficients.setZero();coefficients(2,7)=1.7;coefficients(2,0)=1.e-27;
  height.local_data_.minco_traj_=Trajectory<7>({100.},{coefficients});assert(height.checkTrajHeight());
  height.gcopter_config_->flightMinZ=-1.e6;height.gcopter_config_->flightMaxZ=1.e6;assert(height.checkTrajHeight());
  graph->parallel_bubble_astar_->flight_min_z_=1.35;graph->parallel_bubble_astar_->flight_max_z_=2.15;
  vector<Eigen::Vector3f> heightPath;
  assert(graph->parallel_bubble_astar_->search(Eigen::Vector3f(-2,0,2),Eigen::Vector3f(-2,1,3),heightPath,.01,false,true)==ParallelBubbleAstar::END_FAIL);
  std::cout<<"PASS: navigation one-shot height gate and entire polynomial altitude bounds, including interior overshoot\n";
  {
    // A near-wall accepted start fails the PREFERRED clearance gate, but a
    // certified local departure can reach a safe anchor for native replanning.
    if(!nh.native()->has_parameter("StartCorridorRecovery"))nh.native()->declare_parameter<bool>("StartCorridorRecovery",false);
    if(!nh.native()->has_parameter("StartCorridorRecoveryMargin"))nh.native()->declare_parameter<double>("StartCorridorRecoveryMargin",.1);
    nh.native()->set_parameter(rclcpp::Parameter("StartCorridorRecovery",true));
    nh.native()->set_parameter(rclcpp::Parameter("StartCorridorRecoveryMargin",.05));
    cloud.emplace_back(-4,0,2);scan();
    auto nearStart=std::make_shared<TopoNode>(),remoteGoal=std::make_shared<TopoNode>();
    nearStart->center_=Eigen::Vector3f(-.65,0,2);remoteGoal->center_=Eigen::Vector3f(-6,0,2);
    nearStart->neighbors_.insert(remoteGoal);remoteGoal->neighbors_.insert(nearStart);
    nearStart->paths_[remoteGoal]={nearStart->center_,Eigen::Vector3f(-2,0,2),
        Eigen::Vector3f(-3,0,2),Eigen::Vector3f(-4,0,2),remoteGoal->center_};
    nearStart->weight_[remoteGoal]=remoteGoal->weight_[nearStart]=6.;
    fast_planner::FastSearcher recoveringSearch;recoveringSearch.init(graph,nullptr);
    vector<Eigen::Vector3f> nativeRepaired;
    assert(recoveringSearch.search(nearStart,Eigen::Vector3f::Zero(),remoteGoal,.2,nativeRepaired)==ParallelBubbleAstar::REACH_END);
    assert((nativeRepaired.front()-nearStart->center_).norm()<1.e-5);
    assert((nativeRepaired.back()-remoteGoal->center_).norm()<1.e-5);
    for(size_t j=1;j<nativeRepaired.size();++j) {
      vector<Eigen::Vector3f> segment={nativeRepaired[j-1],nativeRepaired[j]};
      assert(graph->parallel_bubble_astar_->collisionCheck_shortenPath(segment,.5));
    }
    std::cout<<"PASS: occupied cached geometry repaired via certified near-wall departure and native A*; goal unchanged\n";
  }
  {
    auto clock=nh.native()->get_clock()->get_clock_handle();
    assert(rcl_enable_ros_time_override(clock)==RCL_RET_OK);
    assert(rcl_set_ros_time_override(clock,10000000000LL)==RCL_RET_OK);
    manager.frontier_manager_ptr_=make_shared<FrontierManager>();
    if(!nh.native()->has_parameter("exploration.frontier_empty_confirmation"))
      nh.native()->declare_parameter<double>("exploration.frontier_empty_confirmation",3.);
    nh.native()->set_parameter(rclcpp::Parameter("exploration.frontier_empty_confirmation",3.));
    manager.frontier_empty_since_=-1.;assert(manager.frontierStatus()==FRONTIER_PENDING);
    manager.frontier_empty_since_=epic_ros::Time::now().toSec()-3.1;
    assert(manager.frontierStatus()==NO_FRONTIER);
  }
  rclcpp::shutdown();
}
