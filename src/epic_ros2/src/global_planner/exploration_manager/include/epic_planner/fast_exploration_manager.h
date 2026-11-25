/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-21 21:31:51
 * @LastEditTime: 2024-03-10 11:39:33
 * @Description:
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved.
 */

#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <Eigen/Eigen>
#include <frontier_manager/frontier_manager.h>
#include <functional>
#include <memory>
#include <omp.h>
#include <opencv2/opencv.hpp>
#include <pointcloud_topo/graph.h>
#include <plan_manage/planner_manager.h>
#include <epic_ros2/ros.hpp>
#include <vector>
using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class EDTEnvironment;
class SDFMap;
class FastPlannerManager;
struct ExplorationParam;
struct ExplorationData;

enum EXPL_RESULT { NO_FRONTIER, FAIL, START_FAIL, SUCCEED, FRONTIER_PENDING };

class FastExplorationManager {
public:
  typedef shared_ptr<FastExplorationManager> Ptr;
  FastExplorationManager();
  ~FastExplorationManager();
  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  epic_ros::Timer frontier_timer_;
  FrontierManager::Ptr frontier_manager_ptr_;
  double goal_yaw;
  vector<Eigen::Vector3f> active_contour_approach_;
  vector<Eigen::Vector3f> published_first_route_,published_preview_;
  vector<Eigen::Vector3f> active_contour_preview_;
  ClusterInfo::Ptr selected_observation_,published_wall_observation_;
  vector<ClusterInfo::Ptr> deferred_observations_;
  // RViz data from the same solve used for the handoff.
  const char *selection_reason_ = "Reachable observation";
  std::optional<Eigen::Vector3f> restored_observation_;
  double last_observation_search_=-1.e9;
  void rememberDisplacedGoal(const Eigen::Vector3f &body,const vector<Eigen::Vector3f> &replacement);
  void rememberObservation(const ClusterInfo::Ptr &observation);
  void appendDeferredObservation(const Eigen::Vector3f &body,vector<TopoNode::Ptr> &viewpoints);
  std::optional<std::pair<Eigen::Vector3f,Eigen::Vector3f>> published_first_leg_;
  double departure_deadline_=-1.;
  bool right_hand_priority_=false, published_right_hand_=false;
  double right_hand_deadline_=-1., right_hand_progress_time_=-1.;
  float right_hand_best_distance_=0.f;
  // Door crossed on the issued path to a measured wall. A subsequent TSP
  // solve may update entered_turn_ before its replacement is published.
  std::optional<std::pair<Eigen::Vector3f,Eigen::Vector2f>> issued_wall_entry_;
  double issued_wall_entry_until_=-1.;
  std::optional<Eigen::Vector3f> active_sweep_entry_,published_sweep_entry_;
  static bool enteredSweep(const Eigen::Vector3f &body,const Eigen::Vector3f &entry,
                           const Eigen::Vector3f &goal,float tolerance);
  double endpoint_height_margin_ = -1.;
  bool reanchorContour(vector<Eigen::Vector3f> &path,const Eigen::Vector3f &body,bool multi_leg=false);
  static size_t heightBoundedPrefix(const vector<Eigen::Vector3f> &path,double margin);
  bool advanceOpeningGuide(const Eigen::Vector3f &body);
  bool guideSupported(const vector<Eigen::Vector3f> &path) const;
  bool planReturnLeg(const Eigen::Vector3f &start, Eigen::Vector3f &home,
                     vector<Eigen::Vector3f> &route) const;
  static bool passedDuringPlanning(const Eigen::Vector3f &start,const Eigen::Vector3f &goal,
                                   const vector<Eigen::Vector3f> &guide,float radius);
  static bool repeatsReachedGuide(const Eigen::Vector3f &body,
      const vector<Eigen::Vector3f> &issued,const vector<Eigen::Vector3f> &candidate,float tolerance);
  vector<Eigen::Vector3f> publishedGuide() const;
  bool extendRightWallGuide(vector<Eigen::Vector3f> &route,const Eigen::Vector2f &heading) const;
  static double pathTangentYaw(const vector<Eigen::Vector3f> &route,size_t index,float lookahead,double fallback);
  static bool shortensIssuedGuide(const vector<Eigen::Vector3f> &issued,
      const vector<Eigen::Vector3f> &proposed,float tolerance,float minimum_tail);
  static vector<Eigen::Vector3f> appendOnwardGuide(const vector<Eigen::Vector3f> &issued,
      const vector<Eigen::Vector3f> &proposed,const Eigen::Vector3f &goal,float max_gap);
  static vector<Eigen::Vector3f> spliceOnwardGuide(const vector<Eigen::Vector3f> &issued,
      const vector<Eigen::Vector3f> &proposed,const Eigen::Vector3f &goal,float max_gap,
      const std::function<bool(const vector<Eigen::Vector3f>&)> &accept);
  static bool reversesInCruise(const vector<Eigen::Vector3f> &route,const Eigen::Vector3f &goal,
      const Eigen::Vector2f &velocity,float radius,float lookahead=0.f);
  static bool shortOnForwardPass(const Eigen::Vector3f &body,const Eigen::Vector3f &goal,
      const vector<Eigen::Vector3f> &preview,const Eigen::Vector3f &forward_goal,float radius,float horizon);
  static vector<Eigen::Vector3f> shortenedForwardPrefix(const vector<Eigen::Vector3f> &issued,
      const vector<Eigen::Vector3f> &proposal,const Eigen::Vector3f &goal,
      const Eigen::Vector2f &heading,float radius,float braking_horizon,bool retraces_flown_strip=false);
  static vector<Eigen::Vector3f> forwardThroughGoal(const vector<Eigen::Vector3f> &previous,
      const vector<Eigen::Vector3f> &guide,size_t first_goal,const Eigen::Vector3f &velocity,float radius,float accel);
  static bool retractsStraightGoal(const vector<Eigen::Vector3f> &committed,
      const vector<Eigen::Vector3f> &proposed,const vector<Eigen::Vector3f> &preview,float radius);
  bool finishUnflownPass(vector<Eigen::Vector3f> &path,size_t first_goal,const Eigen::Vector3f &velocity);
  bool restorePublishedObservation(vector<Eigen::Vector3f> &approach,vector<Eigen::Vector3f> committed,
                                   vector<Eigen::Vector3f> preview);
  void preservePublishedCorner(vector<Eigen::Vector3f> &approach,const Eigen::Vector3f &velocity);
  static bool skipsWallCorner(const vector<Eigen::Vector3f> &preview,const vector<Eigen::Vector3f> &approach,float radius);
  static bool retracesEntryToWall(const vector<Eigen::Vector3f> &approach,const Eigen::Vector3f &velocity,
                                  const Eigen::Vector3f &entry,const Eigen::Vector2f &inward,float radius);
  void rememberShortRightTurn(const vector<Eigen::Vector3f> &approach,const vector<Eigen::Vector3f> &preview);
  static bool reachesInteriorObservation(const vector<Eigen::Vector3f> &route,const Eigen::Vector3f &goal,
      const Eigen::Vector3f &lip,const Eigen::Vector2f &inward,float radius);
  bool selectedWallInsideTurn(const vector<Eigen::Vector3f> &route,const Eigen::Vector3f &lip,
      const Eigen::Vector2f &inward,float radius) const;
  bool continueShortRightTurn(vector<Eigen::Vector3f> &route);
  std::optional<std::pair<Eigen::Vector3f,Eigen::Vector2f>> short_right_turn_;
  double short_turn_until_=-1.;
  std::optional<std::pair<Eigen::Vector3f,Eigen::Vector2f>> entered_turn_;
  static bool passesBesideEntry(const vector<Eigen::Vector3f> &route,const Eigen::Vector3f &lip,
                                const Eigen::Vector2f &inward,float radius);
  static bool exitsThroughEntry(const vector<Eigen::Vector3f> &route,const Eigen::Vector3f &entry,
                               const Eigen::Vector2f &inward,float radius);
  static bool backtracksRightTurn(const vector<Eigen::Vector3f> &issued,const vector<Eigen::Vector3f> &replacement,float radius);
  bool retractsMeasuredEntry(const vector<Eigen::Vector3f> &replacement);
  bool extendMeasuredTurn(vector<Eigen::Vector3f> &path,size_t first_goal,const Eigen::Vector3f &velocity);
  void limitFoldedPreview(vector<Eigen::Vector3f> &path,size_t first_goal) const;
  bool retracesApproach(const vector<Eigen::Vector3f> &path,size_t first_goal) const;
  bool repairGuide(const Eigen::Vector3f &start,const Eigen::Vector3f &goal,vector<Eigen::Vector3f> &path);
  bool use_velocity_direction_cost_ = false;
  shared_ptr<FastPlannerManager> planner_manager_;
  // ViewpointForest::Ptr vps_forest_;
  double getPathCost(TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1, TopoNode::Ptr &n2, float &yaw2,
                     double *initial_turn_cost = nullptr, vector<Eigen::Vector3f> *tour_path = nullptr);
  static double initialDirectionCost(const vector<Eigen::Vector3f> &path,
                                     const Eigen::Vector3d &velocity,
                                     double weight, double yaw_rate);
  double getPathCostWithoutTopo(TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1, TopoNode::Ptr &n2, float &yaw2);
  void initialize(epic_ros::NodeHandle &nh, FrontierManager::Ptr frt_manager,
                  FastPlannerManager::Ptr planner_manager);
  int frontierStatus(const Eigen::Vector3f &body,bool exhausted_recovery=false);
  int recoverOrWait(const Eigen::Vector3f &body,const Eigen::Vector3f &velocity);
  size_t next_repair_candidate_=0;
  double frontier_empty_since_ = -1.;
  vector<Eigen::Vector3f> exhausted_recovery_route_;
  vector<Eigen::Vector3f> opening_guide_;
  static bool bypassesRightTurn(const Eigen::Vector3f &body,const Eigen::Vector3f &velocity,
      const Eigen::Vector3f &goal,const vector<Eigen::Vector3f> &proposed,float radius);
  int planGlobalPath(const Vector3d &pos, const Vector3d &vel);
  bool solveLkh(const Eigen::MatrixXd &cost_mat, vector<int> &indices);

  void surfaceFrtCalllback(const epic_ros::TimerEvent &e);
  void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
  void updateGoalNode(const TopoNode::Ptr &validated_source = nullptr);
  static void limitPreviewSlope(vector<Eigen::Vector3f> &guide,size_t first_goal,float tolerance);
  bool extendTourPath(vector<Eigen::Vector3f> &path, bool preserve_first_vertex=false);
  bool reanchorLocalPath(vector<Eigen::Vector3f> &path, const Eigen::Vector3f &boundary,
                        const Eigen::Vector3f &velocity);
};

} // namespace fast_planner

#endif
