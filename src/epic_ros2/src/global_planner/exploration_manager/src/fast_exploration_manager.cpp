/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-02-25 15:00:51
 * @LastEditTime: 2024-03-12 22:15:11
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#include <epic_ros2/lkh_tour.hpp>
#include <epic_ros2/planning_workers.hpp>
#include <boost/lexical_cast.hpp>
#include <epic_planner/expl_data.h>
#include <epic_planner/fast_exploration_manager.h>
#include <cerrno>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_manage/planner_manager.h>
#include <spawn.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <visualization_msgs/msg/marker.hpp>
using namespace std;
using namespace Eigen;

extern char **environ;

namespace fast_planner {
namespace {
bool runLkhWorker(const std::filesystem::path &parameters) {
  std::error_code error;
  const auto executable = std::filesystem::read_symlink("/proc/self/exe", error);
  if (error) return false;
  auto worker = (executable.parent_path() / "epic_lkh_worker").string();
  auto input = parameters.string();
  char *argv[] = {worker.data(), input.data(), nullptr};
  pid_t pid;
  // LKH terminates its process on some input and I/O errors.
  if (posix_spawn(&pid, worker.c_str(), nullptr, nullptr, argv, environ) != 0) return false;

  using namespace std::chrono_literals;
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  int status = 0;
  for (;;) {
    const auto result = waitpid(pid, &status, WNOHANG);
    if (result == pid) return WIFEXITED(status) && WEXITSTATUS(status) == 0;
    if (result < 0 && errno != EINTR) return false;
    if (std::chrono::steady_clock::now() >= deadline) break;
    std::this_thread::sleep_for(1ms);
  }
  kill(pid, SIGKILL);
  while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
  return false;
}

float distanceToPolyline(const Vector3f &point,const vector<Vector3f> &path) {
  float best=std::numeric_limits<float>::infinity();
  for (size_t i=1;i<path.size();++i) {
    const Vector3f edge=path[i]-path[i-1];
    const float u=std::clamp((point-path[i-1]).dot(edge)/std::max(.0001f,edge.squaredNorm()),0.f,1.f);
    best=std::min(best,(point-path[i-1]-u*edge).squaredNorm());
  }
  return std::sqrt(best);
}
}
// Test the next few metres only. A certified right turn already underway
// should not be replaced by a distant observation across its entrance.
bool FastExplorationManager::bypassesRightTurn(const Vector3f &body,const Vector3f &velocity,
                              const Vector3f &goal,const vector<Vector3f> &proposed,float radius) {
  const Vector2f forward=velocity.head<2>();
  const Vector3f delta=goal-body;
  if (forward.norm()<.2f || delta.head<2>().norm()<radius) return false;
  bool turning_right=forward.x()*delta.y()-forward.y()*delta.x()<-.3f*forward.norm()*delta.head<2>().norm();
  if (!turning_right && proposed.size()>1) {
    const Vector2f next=(proposed.back()-body).head<2>();
    const auto old=delta.head<2>();
    turning_right=next.norm()>radius && old.x()*next.y()-old.y()*next.x()>.45f*old.norm()*next.norm() &&
                  forward.dot(old)>0.f;
  }
  if (!turning_right) return false;
  if (proposed.size()>1 && delta.norm()<4.f*radius) {
    const Vector2f along=forward.normalized(),right(along.y(),-along.x());
    const float depth=delta.head<2>().dot(right);
    const Vector2f next=(proposed.back()-body).head<2>();
    // Passing the old vertex in the middle of a replacement guide still
    // rounds away from it when the new goal is already back outside the turn.
    if (depth>radius && next.dot(along)>.5f*radius && next.dot(right)<.65f*depth) return true;
  }
  const Vector3f committed=body+delta*std::min(1.f,2.5f*radius/delta.norm());
  float arc=0.f;
  for (size_t i=1;i<proposed.size() && arc<4.f*radius;++i) {
    const Vector3f edge=proposed[i]-proposed[i-1];
    const float u=std::clamp((committed-proposed[i-1]).dot(edge)/std::max(.0001f,edge.squaredNorm()),0.f,1.f);
    if ((committed-proposed[i-1]-u*edge).norm()<.8f*radius) return false;
    arc+=edge.norm();
  }
  return true;
}

// A right-hand boundary already bends away from the current line of flight.
// Require a sustained change inside unflown measured air so that tiny changes
// in the voxelised surface never pin the first observation.
static bool hasBoundaryTurn(const ClusterInfo &observation,const Vector2f &heading,float radius,
                        const FrontierManager &frontiers) {
  const auto &approach=observation.contour_approach_;
  if (approach.size()<2 || heading.norm()<.2f) return false;
  const Vector2f forward=heading.normalized(),right(forward.y(),-forward.x());
  if ((approach[1]-approach[0]).head<2>().norm()>2.f*radius) return false;
  const Vector2f start=approach[0].head<2>();
  Vector2f previous=approach[1].head<2>();
  float travel=0.f,rightward=0.f,leftward=0.f;
  auto along=[&](const Vector3f &point) {
    const Vector2f delta=point.head<2>()-previous;
    previous=point.head<2>();
    const float length=delta.norm();
    if (length<.025f) return false;
    travel+=length;
    const float lateral=delta.dot(right);
    if (!frontiers.visitedStance(point)) {
      rightward=std::max(0.f,rightward+lateral);
      leftward=std::max(0.f,leftward-lateral);
    }
    const float offset=(previous-start).dot(right);
    return travel<=7.f*radius && ((rightward>radius && offset>.8f*radius) ||
                                 (leftward>radius && offset<-.8f*radius));
  };
  for (size_t j=2;j<approach.size() && travel<=7.f*radius;++j) if (along(approach[j])) return true;
  for (size_t j=1;j<observation.contour_preview_.size() && travel<=7.f*radius;++j)
    if (along(observation.contour_preview_[j])) return true;
  return false;
}

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
  nh.getParam("view_graph", ep_->view_graph_);
  nh.param("ViewpointManager/local_tsp_size", ep_->local_viewpoint_num_, 12);
  nh.getParam("global_planning/w_vdir", ep_->w_vdir_);
  nh.param("bubble_astar/endpoint_height_margin",endpoint_height_margin_,-1.);
  nh.param("global_planning/right_hand_priority",right_hand_priority_,false);
  bool external;
  nh.param("fsm/external_local_planner",external,false);
  nh.param("global_planning/use_velocity_direction_cost",use_velocity_direction_cost_,external);
  if(use_velocity_direction_cost_)ROS_INFO("Global first-leg direction cost enabled: w=%.2f, native LKH retained",ep_->w_vdir_);
  nh.getParam("global_planning/w_yawdir", ep_->w_yawdir_);
  Eigen::Vector3d origin, size;
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

// A distant opening can require a descent followed by an ascent. Apply the
// local endpoint envelope to one continuous leg of the measured connector.
size_t FastExplorationManager::heightBoundedPrefix(const vector<Eigen::Vector3f> &path,double margin) {
  return FrontierManager::heightBoundedPrefix(path,margin);
}

bool FastExplorationManager::guideSupported(const vector<Eigen::Vector3f> &path) const {
  if (!planner_manager_->lidar_map_interface_->IsPathInBox(path)) return false;
  if (!frontier_manager_ptr_->surface_continuity_) return true;
  return std::all_of(path.begin(),path.end(),[&](const auto &p){return frontier_manager_ptr_->withinFlightHeight(p);}) &&
      heightBoundedPrefix(path,endpoint_height_margin_)==path.size() && frontier_manager_ptr_->observedGuide(path);
}

bool FastExplorationManager::planReturnLeg(const Vector3f &start, Vector3f &home,
                                           vector<Vector3f> &route) const {
  auto &front = *frontier_manager_ptr_;
  auto &map = *planner_manager_->lidar_map_interface_;
  auto &astar = *planner_manager_->topo_graph_->parallel_bubble_astar_;
  const float step = .5f * front.getCellSize();
  const auto valid_home = [&]() {
    return front.withinFlightHeight(home) && map.IsInBox(home) && front.observedAir(home) &&
        map.getDisToOcc(home) > astar.safe_distance_ + .01;
  };
  // Preserve the launch XY if the first airborne height is too close to the floor.
  const float max_home_z = std::min(float(astar.flight_max_z_), home.z() + float(astar.safe_distance_));
  while (!valid_home() && home.z() < max_home_z) home.z() = std::min(home.z() + step, max_home_z);
  if (!valid_home()) return false;

  const auto certified = [&](const vector<Vector3f> &candidate) {
    return map.IsPathInBox(candidate) && front.certifyFlightGuide(candidate);
  };
  route = {start, home};
  if (!certified(route) &&
      !(front.connectObserved(start, home, .10, route) && certified(route)) &&
      !(front.connectFlightHistory(start, home, .18, route) && certified(route)) &&
      !(front.connectDistantObserved(start, home, route) && certified(route))) return false;

  // Keep the local planner's altitude envelope valid on a return across levels.
  route.resize(heightBoundedPrefix(route, endpoint_height_margin_));
  float remaining = front.sensorUpdateRange();
  for (size_t i = 1; i < route.size(); ++i) {
    const Vector3f edge = route[i] - route[i - 1];
    const float length = edge.norm();
    if (length > remaining) {
      route[i] = route[i - 1] + (remaining / length) * edge;
      route.resize(i + 1);
      break;
    }
    remaining -= length;
  }
  return certified(route);
}

bool FastExplorationManager::passedDuringPlanning(const Vector3f &start,const Vector3f &goal,
                                                  const vector<Vector3f> &guide,float radius) {
  if (guide.size()<2 || radius<=0.f) return false;
  const Vector3f travelled=guide.front()-start;
  const float length=travelled.norm();
  if (length<=radius || std::abs(travelled.z())>radius) return false;
  const Vector3f direction=travelled/length;
  const float along=(goal-start).dot(direction);
  if (along<.5f*radius || along>length-.5f*radius) return false;
  // Only discard a handoff entirely inside the strip just crossed. A side
  // entrance, an onward preview or a genuine return beyond it remains valid.
  const auto crossed=[&](const Vector3f &point) {
    const Vector3f delta=point-start;
    return (delta-std::clamp(delta.dot(direction),0.f,length)*direction).squaredNorm()<=radius*radius;
  };
  return crossed(goal) && std::all_of(guide.begin(),guide.end(),crossed);
}

bool FastExplorationManager::repeatsReachedGuide(const Vector3f &body,
    const vector<Vector3f> &issued,const vector<Vector3f> &candidate,float tolerance) {
  if (issued.size()<2 || candidate.empty()) return false;
  const float tolerance_squared=tolerance*tolerance;
  const auto reached=[&](const Vector3f &point) {
    return (point-body).squaredNorm()<=tolerance_squared;
  };
  // Match a previously issued endpoint and keep previews that leave it.
  return reached(issued.back()) && std::all_of(candidate.begin(),candidate.end(),reached);
}

double FastExplorationManager::pathTangentYaw(const vector<Vector3f> &route,
    size_t index,float lookahead,double fallback) {
  if (index>=route.size()) return fallback;
  Vector2f heading=Vector2f::Zero();
  float remaining=lookahead;
  for (size_t i=index+1;i<route.size() && remaining>0.f;++i) {
    const Vector2f edge=(route[i]-route[i-1]).head<2>();
    const float length=edge.norm();
    if (length<.01f) continue;
    if (heading.dot(edge)<0.f) break;
    const float distance=std::min(length,remaining);
    heading+=(distance/length)*edge;
    remaining-=distance;
  }
  return heading.norm()>.05f ? std::atan2(heading.y(),heading.x()) : fallback;
}

vector<Vector3f> FastExplorationManager::publishedGuide() const {
  auto guide=published_first_route_;
  if (!guide.empty() && published_preview_.size()>1 && (guide.back()-published_preview_.front()).norm()<.1f)
    guide.insert(guide.end(),published_preview_.begin()+1,published_preview_.end());
  return guide;
}

bool FastExplorationManager::shortensIssuedGuide(const vector<Vector3f> &issued,
    const vector<Vector3f> &proposed,float tolerance,float minimum_tail) {
  if (issued.size()<2 || proposed.size()<2) return false;
  float progress=0.f,total=0.f;
  for (size_t i=1;i<issued.size();++i) total+=(issued[i]-issued[i-1]).norm();
  for (size_t i=1;i<proposed.size();++i) {
    float arc=0.f,nearest=tolerance*tolerance,projected=-1.f;
    for (size_t j=1;j<issued.size();++j) {
      const Vector3f edge=issued[j]-issued[j-1];
      const float length=edge.norm();
      if (length<.001f) continue;
      const float t=std::clamp((proposed[i]-issued[j-1]).dot(edge)/edge.squaredNorm(),0.f,1.f);
      const float at=arc+t*length;
      const float error=(proposed[i]-issued[j-1]-t*edge).squaredNorm();
      if (at+tolerance>=progress && error<nearest) {
        nearest=error;
        projected=at;
      }
      arc+=length;
    }
    if (projected<0.f) return false;
    progress=std::max(progress,projected);
  }
  return total-progress>=minimum_tail;
}

bool FastExplorationManager::extendRightWallGuide(vector<Vector3f> &route,const Vector2f &heading) const {
  if (route.empty()) return false;
  auto &front=*frontier_manager_ptr_;
  const auto preview=front.previewRightWall(route.back(),heading);
  if (preview.size()<2) return false;
  auto extended=route;
  extended.insert(extended.end(),preview.begin()+1,preview.end());
  if (!guideSupported(extended) || !front.certifyFlightGuide(extended) ||
      front.revisitsLowerPass(route.front(),extended)) return false;
  route.swap(extended);
  return true;
}

vector<Vector3f> FastExplorationManager::appendOnwardGuide(const vector<Vector3f> &issued,
    const vector<Vector3f> &proposed,const Vector3f &goal,float max_gap) {
  if (issued.size()<2 || proposed.size()<2) return {};
  const auto &tip=issued.back();
  const Vector3f incoming=tip-issued[issued.size()-2];
  if (incoming.norm()<.01f) return {};
  const auto target=std::find_if(proposed.begin()+1,proposed.end(),[&](const auto &point) {
    return (point-goal).norm()<.01f;
  });
  if (target==proposed.end() || (target+1==proposed.end() && (tip-*target).norm()<=.01f)) return {};
  for (auto join=target;join!=proposed.begin();--join) {
    const Vector3f gap=*join-tip;
    if (gap.norm()>max_gap) continue;
    if (gap.norm()>.01f && incoming.dot(gap)<-.15f*incoming.norm()*gap.norm()) continue;
    if (join+1!=proposed.end()) {
      const Vector3f outgoing=*(join+1)-*join;
      if (incoming.dot(outgoing)<-.15f*incoming.norm()*outgoing.norm() ||
          gap.dot(outgoing)<-.15f*gap.norm()*outgoing.norm()) continue;
    }
    const bool shared_vertex=gap.norm()<=.01f;
    auto extended=issued;
    extended.insert(extended.end(),join+shared_vertex,proposed.end());
    return extended;
  }
  return {};
}

vector<Vector3f> FastExplorationManager::spliceOnwardGuide(const vector<Vector3f> &issued,
    const vector<Vector3f> &proposed,const Vector3f &goal,float max_gap,
    const std::function<bool(const vector<Vector3f>&)> &accept) {
  if (issued.size()<2 || proposed.size()<2) return {};
  const auto target=std::find_if(proposed.begin()+1,proposed.end(),[&](const auto &point) {
    return (point-goal).norm()<.01f;
  });
  if (target==proposed.end()) return {};

  constexpr float sample_spacing=.25f;
  float tail_length=0.f;
  int checks=0;
  // Try the latest departure first and preserve all but a short tail of the issued route.
  for (size_t end=issued.size()-1;end>0 && tail_length<2.f*max_gap;--end) {
    const Vector3f edge=issued[end]-issued[end-1];
    const float length=edge.norm();
    if (length<.01f) continue;
    const Vector3f incoming=edge/length;
    const int steps=std::max(2,int(std::ceil(length/sample_spacing)));
    for (int step=1;step<steps;++step) {
      const float trim=length*step/steps;
      if (tail_length+trim>2.f*max_gap) break;
      const Vector3f start=issued[end]-trim*incoming;
      for (auto next=target;next!=proposed.begin();--next) {
        const Vector3f proposed_edge=*next-*(next-1);
        const float proposed_length=proposed_edge.norm();
        if (proposed_length<.01f) continue;
        const Vector3f outgoing=proposed_edge/proposed_length;
        if (incoming.dot(outgoing)<-.15f) continue;
        const int parts=std::max(2,int(std::ceil(proposed_length/sample_spacing)));
        for (int part=1;part<=parts;++part) {
          const float remaining=proposed_length*part/parts;
          const Vector3f finish=*next-remaining*outgoing;
          const Vector3f gap=finish-start;
          const float distance=gap.norm();
          if (distance<.05f || distance>max_gap ||
              incoming.dot(gap)<0.f || outgoing.dot(gap)<0.f) continue;

          // Match both tangents and keep the control points inside the join.
          const float handle=std::min(distance/3.f,remaining);
          const Vector3f a=start+handle*incoming,b=finish-handle*outgoing;
          vector<Vector3f> joined(issued.begin(),issued.begin()+end);
          joined.push_back(start);
          for (int sample=1;sample<=4;++sample) {
            const float t=sample/4.f,u=1.f-t;
            joined.push_back(u*u*u*start+3.f*u*u*t*a+3.f*u*t*t*b+t*t*t*finish);
          }
          joined.insert(joined.end(),next,proposed.end());
          if (accept(joined)) return joined;
          if (++checks>=8) return {};
        }
      }
    }
    tail_length+=length;
  }
  return {};
}

// A complete replacement that ends behind the moving body is different
// from an onward preview through a close observation. At a real dead end
// SUPER may finish braking on the active guide before accepting a return.
bool FastExplorationManager::reversesInCruise(const vector<Vector3f> &route,const Vector3f &goal,
    const Vector2f &velocity,float radius,float lookahead) {
  const float speed=velocity.norm();
  // Use the same settled-motion threshold as the external execution FSM.
  // A short planning period does not make a backwards command admissible
  // when odometry still shows motion over a previously swept strip.
  if (route.size()<2 || radius<=0.f || speed<.35f) return false;
  const Vector2f heading=velocity/speed;
  const auto lead=[&](const Vector3f &p){return (p-route.front()).head<2>().dot(heading);};
  const float tip=lead(route.back());
  if (lead(goal)<.5f*radius && tip<-.1f*radius) return true;
  if (lookahead>0.f && (goal-route.front()).norm()<radius &&
      std::all_of(route.begin()+1,route.end(),[&](const Vector3f &p) {
        return (p-route.front()).squaredNorm()<radius*radius;
      })) return true;
  if (lookahead<=0.f || lead(goal)>lookahead || tip>lookahead) return false;
  // A close #1 may still be ahead at publication while its guide doubles
  // back before the next solve. Keep genuine lateral/vertical turns.
  float peak=0.f;
  for (const auto &p:route) {
    const Vector3f delta=p-route.front();
    if (std::abs(delta.z())>radius || (delta.head<2>()-lead(p)*heading).norm()>1.5f*radius) return false;
    peak=std::max(peak,lead(p));
  }
  return peak-tip>radius;
}

// A farther measured observation defines the actual wall direction. Using
// the direction of an older turn here can make two points on one wall look
// like different passages, especially immediately after entering an aisle.
bool FastExplorationManager::shortOnForwardPass(const Vector3f &body,const Vector3f &goal,
    const vector<Vector3f> &preview,const Vector3f &forward_goal,float radius,float horizon) {
  const Vector2f extent=(forward_goal-body).head<2>();
  if (extent.norm()<=horizon+2.f*radius) return false;
  const Vector2f axis=extent.normalized();
  const auto along=[&](const Vector3f &p){return (p-body).head<2>().dot(axis);};
  const auto on_strip=[&](const Vector3f &p) {
    return std::abs(p.z()-body.z())<=radius &&
        ((p-body).head<2>()-along(p)*axis).norm()<=2.f*radius;
  };
  if (!on_strip(goal) || along(goal)>=horizon) return false;
  // Preserve an actual lateral exit or a preview that carries flight forward.
  return std::all_of(preview.begin(),preview.end(),on_strip) &&
         (preview.empty() || along(preview.back())<horizon || along(goal)<-radius);
}

// Return the part of the issued approach that a short replacement would
// discard. Use the local tangent in the direction of flight, since the
// destination of the same approach may be around several corners.
vector<Vector3f> FastExplorationManager::shortenedForwardPrefix(const vector<Vector3f> &issued,
    const vector<Vector3f> &proposal,const Vector3f &goal,const Vector2f &heading,
    float radius,float braking_horizon,bool retraces_flown_strip) {
  if (issued.size()<2 || proposal.size()<2 || radius<=0.f || heading.norm()<.01f) return {};
  const Vector3f body=proposal.front();
  const Vector3f axis=Vector3f(heading.x(),heading.y(),0.f).normalized();
  const auto near_strip=[&](const Vector3f &point) {
    const Vector3f delta=point-body;
    return std::abs(delta.z())<=radius &&
        (delta.head<2>()-delta.dot(axis)*axis.head<2>()).norm()<=1.5f*radius;
  };
  const float goal_lead=(goal-body).dot(axis);
  const bool returning_over_flight=goal_lead<0.f && retraces_flown_strip;
  if ((!returning_over_flight && !near_strip(goal)) || goal_lead>braking_horizon) return {};
  float new_lead=0.f;
  for (const auto &point:proposal) {
    if (!near_strip(point) && !returning_over_flight) return {};
    new_lead=std::max(new_lead,(point-body).dot(axis));
  }
  const float needed=std::max(3.f*radius,new_lead+2.f*radius);
  size_t next=0;
  float nearest=radius*radius;
  Vector3f projection=body;
  for (size_t i=1;i<issued.size();++i) {
    const Vector3f edge=issued[i]-issued[i-1];
    if (edge.squaredNorm()<1.e-6f) continue;
    const float t=std::clamp((body-issued[i-1]).dot(edge)/edge.squaredNorm(),0.f,1.f);
    const Vector3f point=issued[i-1]+t*edge;
    const float distance=(body-point).squaredNorm();
    if (distance<=nearest) {nearest=distance; next=i; projection=point;}
  }
  if (!next) return {};
  vector<Vector3f> prefix{body};
  if ((projection-body).norm()>.01f) prefix.push_back(projection);
  float lead=(projection-body).dot(axis);
  for (size_t i=next;i<issued.size();++i) {
    const float end=(issued[i]-body).dot(axis);
    if (end<lead-.25f*radius) return {};
    Vector3f tip=issued[i];
    if (end>=needed && end>lead)
      tip=prefix.back()+((needed-lead)/(end-lead))*(issued[i]-prefix.back());
    if (!near_strip(tip)) return {};
    if ((tip-prefix.back()).norm()>.01f) prefix.push_back(tip);
    if (end>=needed) return prefix;
    lead=end;
  }
  return {};
}

// When #1 lies straight ahead, the approach and its preview can both fold
// back across it. Pass through the selected point onto the already issued
// straight only if the complete replacement can be certified on the live map.
vector<Vector3f> FastExplorationManager::forwardThroughGoal(const vector<Vector3f> &previous,
    const vector<Vector3f> &guide,size_t first_goal,const Vector3f &velocity,float radius,float accel) {
  const float speed=velocity.head<2>().norm();
  if (previous.size()<2 || !first_goal || first_goal>=guide.size() || speed<1.f || accel<=0.f) return {};
  const Vector3f body=guide.front(),goal=guide[first_goal];
  const Vector3f axis(velocity.x()/speed,velocity.y()/speed,0.f);
  const float ahead=(goal-body).dot(axis);
  if (ahead<=0.f || ahead>speed*speed/(2.f*accel)+2.f*radius) return {};
  float peak=0.f;
  for (const auto &point:guide) {
    const Vector3f offset=point-body;
    const float along=offset.dot(axis);
    if (along < -2.f*radius || (offset-along*axis).norm()>radius) return {};
    peak=std::max(peak,along);
  }
  if (peak-(guide.back()-body).dot(axis)<radius) return {};
  const float lookahead=std::max(3.f*radius,peak+2.f*radius);
  if (distanceToPolyline(body+.5f*lookahead*axis,previous)>radius) return {};
  for (size_t i=1;i<previous.size();++i) {
    const Vector3f edge=previous[i]-previous[i-1];
    const float begin=(previous[i-1]-body).dot(axis),span=edge.dot(axis);
    if (begin>lookahead || span<=0.f || begin+span<lookahead) continue;
    const Vector3f tip=previous[i-1]+((lookahead-begin)/span)*edge;
    if ((tip-body-lookahead*axis).norm()<=radius) return {body,goal,tip};
  }
  return {};
}

bool FastExplorationManager::reanchorContour(vector<Eigen::Vector3f> &route,const Eigen::Vector3f &body,bool multi_leg) {
  if (route.size()<2) return false;
  // A freshly measured connector can start inside the departure envelope.
  // Splitting its first edge can turn a valid escape into a short edge
  // which ends before the vehicle has regained the cruise clearance.
  if (multi_leg && (route.front()-body).squaredNorm()<1.e-10f)
    return frontier_manager_ptr_->certifyFlightGuide(route);
  if (route.size()==2 && frontier_manager_ptr_->certifyFlightGuide({body,route.back()})) {
    route.front()=body;
    return true;
  }
  float best=std::numeric_limits<float>::infinity(),arc=0.f;
  size_t target=1;
  Eigen::Vector3f projection=route.front();
  const float search_length=6.f+(body-route.front()).norm();
  for (size_t i=1;i<route.size() && arc<search_length;++i) {
    const Eigen::Vector3f delta=route[i]-route[i-1];
    const float length=delta.norm();
    if (length<.001f) continue;
    const float t=std::clamp((body-route[i-1]).dot(delta)/length,0.f,std::min(length,search_length-arc));
    const Eigen::Vector3f point=route[i-1]+(t/length)*delta;
    const float distance=(body-point).norm();
    if (distance<best) {best=distance;target=i;projection=point;}
    arc+=length;
  }
  if (best>2.f) return false;
  vector<Eigen::Vector3f> connector;
  if (!frontier_manager_ptr_->connectObserved(body,route[target],.012,connector)) {
    if (!frontier_manager_ptr_->connectObserved(body,projection,.012,connector)) return false;
    connector.push_back(route[target]);
  }
  connector.insert(connector.end(),route.begin()+target+1,route.end());
  // A new wall sample sometimes joins the outline behind the moving body,
  // then passes back by the body on the very same strip. The closest point
  // on this folded polyline is its departure, even after the drone has
  // already flown the join. Splice only across a narrow, flown excursion.
  if (!multi_leg && connector.size()>2) {
    const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
    size_t rejoin=0;
    Eigen::Vector3f peak=body;
    float travelled=0.f,extent=0.f;
    for (size_t i=1;i<connector.size() && travelled<2.f*frontier_manager_ptr_->sensorUpdateRange();++i) {
      const Eigen::Vector3f delta=connector[i]-connector[i-1];
      if (delta.squaredNorm()<.0001f) continue;
      const float t=std::clamp((body-connector[i-1]).dot(delta)/delta.squaredNorm(),0.f,1.f);
      const Eigen::Vector3f nearest=connector[i-1]+t*delta;
      if (extent>radius && (nearest-body).norm()<radius &&
          travelled+t*delta.norm()>1.65f*extent && frontier_manager_ptr_->visitedStance(peak)) {
        const Eigen::Vector3f axis=(peak-body)/extent;
        bool same_strip=true;
        for (size_t j=1;j<i && same_strip;++j) {
          const Eigen::Vector3f offset=connector[j]-body;
          const float lateral=(offset-std::clamp(offset.dot(axis),0.f,extent)*axis).norm();
          same_strip=lateral<radius || (lateral<1.5f*radius && frontier_manager_ptr_->visitedStance(connector[j]));
        }
        if (same_strip && frontier_manager_ptr_->certifyFlightGuide({body,connector[i]})) rejoin=i;
      }
      travelled+=delta.norm();
      const float distance=(connector[i]-body).norm();
      if (distance>extent) {extent=distance;peak=connector[i];}
    }
    if (rejoin>1) connector.erase(connector.begin()+1,connector.begin()+rejoin);
  }
  if ((!multi_leg && heightBoundedPrefix(connector,endpoint_height_margin_)!=connector.size()) ||
      !frontier_manager_ptr_->certifyFlightGuide(connector)) return false;
  route.swap(connector);
  return true;
}

bool FastExplorationManager::repairGuide(const Eigen::Vector3f &start,const Eigen::Vector3f &goal,
                                         vector<Eigen::Vector3f> &path) {
  if (guideSupported(path)) return true;
  auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
  vector<Eigen::Vector3f> replacement;
  const bool native=astar->search(start,goal,replacement,.05,false)==ParallelBubbleAstar::REACH_END &&
                    astar->collisionCheck_shortenPath(replacement) && guideSupported(replacement);
  if (!native && (!frontier_manager_ptr_->connectFlightHistory(start,goal,.04,replacement) ||
                  !guideSupported(replacement))) return false;
  path.swap(replacement);
  return true;
}

double FastExplorationManager::getPathCost(TopoNode::Ptr &n1,
                                           Eigen::Vector3d v1, float &yaw1,
                                           TopoNode::Ptr &n2, float &yaw2,
                                           double *initial_turn_cost,
                                           vector<Eigen::Vector3f> *tour_path) {
  if(initial_turn_cost)*initial_turn_cost=0.;
  auto estimateCost = [&](TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1,
                          TopoNode::Ptr &n2, float &yaw2, int res,
                          vector<Eigen::Vector3f> &path) -> double {
    double len_cost, yaw_cost, dir_cost;
    len_cost = yaw_cost = dir_cost = 0.0;
    if (res != BubbleAstar::REACH_END || path.size() < 2)
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
  if (frontier_manager_ptr_->surface_continuity_ && (!guideSupported(path) ||
      (n1==planner_manager_->topo_graph_->odom_node_ &&
       frontier_manager_ptr_->revisitsLowerPass(n1->center_,path)))) return 2e3+1.;
  const double cost=estimateCost(n1, v1, yaw1, n2, yaw2, res, path);
  if(initial_turn_cost && cost<2e3)
    *initial_turn_cost=initialDirectionCost(path,v1,ep_->w_vdir_,
                                          planner_manager_->gcopter_config_->yaw_max_vel);
  if (tour_path && cost<2e3) *tour_path=std::move(path);
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

int FastExplorationManager::frontierStatus(const Eigen::Vector3f &body,bool exhausted_recovery) {
  if (std::getenv("EPIC_TRACE_WALL_CHOICE") && frontier_manager_ptr_->surface_continuity_) {
    ROS_WARN_THROTTLE(1., "No executable guide at %.2f %.2f %.2f: body clearance %.3fm, observed air %d",
        body.x(),body.y(),body.z(),planner_manager_->lidar_map_interface_->getDisToOcc(body),
        int(frontier_manager_ptr_->observedAir(body)));
  }
  if (!exhausted_recovery && frontier_manager_ptr_->hasPendingSurveyFrontiers(body)) {
    frontier_empty_since_=-1.;return FRONTIER_PENDING;
  }
  const double now=epic_ros::Time::now().toSec();
  if(frontier_empty_since_<0.)frontier_empty_since_=now;
  epic_ros::NodeHandle nh;double confirm;
  // Debounce survey completion across scan updates.
  nh.param("exploration/frontier_empty_confirmation",confirm,
           frontier_manager_ptr_->surface_continuity_ ? 3. : 0.);
  return now-frontier_empty_since_<confirm ? FRONTIER_PENDING : NO_FRONTIER;
}

bool FastExplorationManager::advanceOpeningGuide(const Eigen::Vector3f &body) {
  const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
  // Recovery can run between a first handoff and SUPER's first feasible turn.
  // Compare the contiguous unflown ends of both measured guides before
  // abandoning a newly issued observation for a different opening.
  if (opening_guide_.size()>1 && published_first_leg_ &&
      published_first_route_.size()>1 && epic_ros::Time::now().toSec()<departure_deadline_ &&
      distanceToPolyline(published_first_leg_->second,opening_guide_)>2.f*radius) {
    const auto fresh_tail=[&](const vector<Vector3f> &route) {
      float length=0.f;
      for (size_t i=route.size()-1;i>0;--i) {
        if (frontier_manager_ptr_->visitedStance(route[i])) break;
        const Vector3f edge=route[i-1]-route[i];
        const int steps=std::max(1,int(std::ceil(edge.norm()/frontier_manager_ptr_->getCellSize())));
        const float step_length=edge.norm()/steps;
        for (int j=0;j<steps;++j) {
          if (frontier_manager_ptr_->visitedStance(route[i]+edge*((j+.5f)/steps))) return length;
          length+=step_length;
        }
      }
      return length;
    };
    const float published_fresh=fresh_tail(published_first_route_);
    if (published_fresh>3.f*radius && published_fresh>fresh_tail(opening_guide_)+2.f*radius) {
      auto proposal=opening_guide_;
      proposal.front()=body;
      if (restorePublishedObservation(proposal,published_first_route_,published_preview_)) {
        opening_guide_.clear();
        frontier_empty_since_=-1.;
        ROS_INFO("Retained newly issued observation with %.2fm unflown through recovery",published_fresh);
        return true;
      }
    }
  }
  if (opening_guide_.size()<2 || (opening_guide_.back()-body).norm()<=.7f*radius ||
      !reanchorContour(opening_guide_,body,true)) return false;
  frontier_manager_ptr_->continueBeforeLeftTurn(body,opening_guide_);
  if (frontier_manager_ptr_->revisitsLowerPass(body,opening_guide_)) return false;
  size_t end=heightBoundedPrefix(opening_guide_,endpoint_height_margin_);
  if (opening_guide_[end-1].z()>body.z()+radius) {
    vector<Eigen::Vector3f> level;
    if (frontier_manager_ptr_->pendingLevelGuide(body,level)) {
      opening_guide_=std::move(level);
      end=heightBoundedPrefix(opening_guide_,endpoint_height_margin_);
    }
  }
  active_contour_approach_.assign(opening_guide_.begin(),opening_guide_.begin()+end);
  if (!frontier_manager_ptr_->certifyFlightGuide(active_contour_approach_)) return false;
  active_contour_preview_.clear();
  frontier_manager_ptr_->perimeter_target_.reset();
  const auto goal=active_contour_approach_.back();
  const auto edge=goal-active_contour_approach_[end-2];
  ed_->global_tour_={body,goal};
  selection_reason_="Continue through measured free space";
  ed_->global_tour_yaws_={planner_manager_->local_data_.curr_yaw_,std::atan2(edge.y(),edge.x())};
  updateGoalNode();
  frontier_empty_since_=-1.;
  return true;
}

int FastExplorationManager::recoverOrWait(const Eigen::Vector3f &body,const Eigen::Vector3f &velocity) {
  const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
  auto &front=*frontier_manager_ptr_;
  if (right_hand_priority_ && front.surface_continuity_) {
    Vector2f heading=velocity.head<2>();
    if (heading.norm()<.5f) {
      const double yaw=planner_manager_->local_data_.curr_yaw_;
      heading=Vector2f(std::cos(yaw),std::sin(yaw));
    }
    vector<Vector3f> wall{body};
    if (extendRightWallGuide(wall,heading)) {
      opening_guide_=std::move(wall);
      if (advanceOpeningGuide(body)) return SUCCEED;
      opening_guide_.clear();
    }
  }
  vector<Eigen::Vector3f> route;
  // Resume a recovery connector only after this solve exhausted the
  // reachable observations. A fresh wall or doorway gets the first choice.
  if (front.surface_continuity_ && !opening_guide_.empty()) {
    if (advanceOpeningGuide(body)) return SUCCEED;
    opening_guide_.clear();
  }
  // Accept a recovery proposal only after the executable first leg survives
  // reanchoring and the height envelope. A rejected first proposal leaves
  // the other measured observations eligible in this same solve.
  const auto commit=[&]() {
    if (route.size()<2 || front.surveyedEnclosureStance(route.back())) return false;
    const Eigen::Vector2f motion=velocity.head<2>();
    const Eigen::Vector2f proposed=(route.back()-body).head<2>();
    const auto reverses=[&](const Eigen::Vector2f &step) {
      return step.dot(motion)<-.2f*step.norm()*motion.norm();
    };
    const auto starts_backwards=[&](const vector<Vector3f> &guide) {
      for (const auto &point:guide) {
        const Vector2f step=(point-body).head<2>();
        if (step.norm()>=radius) return reverses(step);
      }
      return false;
    };
    if (front.surface_continuity_ && motion.norm()>.5f && proposed.norm()>4.f*radius &&
        (starts_backwards(route) || reverses(proposed))) {
      vector<Eigen::Vector3f> ahead;
      if (front.observedBoundaryGuide(body,ahead,&motion,nullptr,2.f*radius) && !starts_backwards(ahead)) {
        opening_guide_=std::move(ahead);
        if (advanceOpeningGuide(body)) return true;
      }
    }
    opening_guide_=std::move(route);
    if (advanceOpeningGuide(body)) return true;
    opening_guide_.clear();
    active_contour_approach_.clear();
    return false;
  };
  vector<Eigen::Vector3f> prefix;
  // A missing local viewpoint can occur halfway along a measured passage.
  // Test the remaining connected observations at this height before taking
  // a recovery branch, even when no upstairs viewpoint entered the tour.
  if ((front.surface_continuity_ && front.pendingLevelGuide(body,route) && commit()) ||
      (front.surface_continuity_ && front.reachableOpening(body,route) && commit()) ||
      (front.sightlineGoal(body,route) && commit()))
    return SUCCEED;
  if (right_hand_priority_ && front.surface_continuity_ && !published_first_leg_) {
    const double yaw=planner_manager_->local_data_.curr_yaw_-M_PI/6.;
    const Vector2f forward_right(std::cos(yaw),std::sin(yaw));
    if (front.observedBoundaryGuide(body,route,&forward_right) && commit()) return SUCCEED;
  }
  if (front.surface_continuity_ && front.observedBoundaryGuide(body,route,nullptr,nullptr,0.f,&prefix)) {
    if (front.exhaustedAboveFlownFloor(route)) {
      exhausted_recovery_route_=route;
      ROS_INFO("Confirming survey completion above a flown floor with no remaining surface viewpoint");
      return frontierStatus(body,true);
    }
    if (commit()) return SUCCEED;
  }
  // The remainder can be remeasured from the far end of this verified leg.
  // The disconnected suffix never enters the trajectory sent to SUPER.
  if (!prefix.empty()) {
    route=std::move(prefix);
    if (commit()) return SUCCEED;
  }
  if (front.surface_continuity_ && front.ground_clearance_>0. && front.risingBoundaryGuide(body,route) && commit()) return SUCCEED;
  if (front.surface_continuity_ && front.previousOpening(body,route) && commit()) return SUCCEED;
  if (front.surface_continuity_ && front.resumeSurveyThroughAperture(body,route) && commit()) return SUCCEED;
  if (front.surface_continuity_ && front.clearanceDepartureGuide(body,opening_guide_)) {
    if (advanceOpeningGuide(body)) return SUCCEED;
    opening_guide_.clear();
  }
  if (front.surface_continuity_ && !published_first_leg_) {
    if (front.initialSurveyGuide(body,opening_guide_) && advanceOpeningGuide(body)) return SUCCEED;
    opening_guide_.clear();
    return FRONTIER_PENDING;
  }
  return frontierStatus(body);
}

bool FastExplorationManager::retractsStraightGoal(const vector<Eigen::Vector3f> &committed,
    const vector<Eigen::Vector3f> &proposed,const vector<Eigen::Vector3f> &preview,float radius) {
  if (committed.size()<2 || proposed.size()<2) return false;
  const Eigen::Vector3f body=proposed.front(),span=committed.back()-body;
  const float length=span.norm();
  if (length<2.f*radius) return false;
  const Eigen::Vector3f axis=span/length;
  const auto in_band=[&](const Eigen::Vector3f &point) {
    const Eigen::Vector3f offset=point-body;
    const float along=offset.dot(axis);
    return (offset-along*axis).norm()<=radius;
  };
  // A new turn or outgoing preview is a useful revision. Only suppress a
  // goal that shortens a currently clear straight without adding an exit.
  const float progress=(proposed.back()-body).dot(axis);
  if (progress>length-1.5f*radius) return false;
  if (!std::all_of(committed.begin(),committed.end(),in_band) ||
      !std::all_of(proposed.begin(),proposed.end(),in_band)) return false;
  for (const auto &point:preview)
    if (!in_band(point) || (point-body).dot(axis)>length-radius) return false;
  return true;
}

// A dynamical handoff may skip #1 while SUPER is still flying the previous
// trajectory. Keep that exact observation available if its real wall is unseen.
void FastExplorationManager::rememberDisplacedGoal(const Vector3f &body,const vector<Vector3f> &replacement) {
  auto &front=*frontier_manager_ptr_;
  const auto &issued=published_first_route_;
  if (issued.size()<2 || replacement.size()<2 || front.exteriorPassPending()) return;
  const auto &q=issued.back();
  const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
  if (std::abs(q.z()-body.z())>radius || (q-body).norm()<3.f*radius ||
      distanceToPolyline(q,replacement)<2.f*radius || front.visitedStance(q) ||
      front.flownObservationStance(q) || front.nearFlightPath(q,1.5f*radius,0.f,.35f*radius)) return;
  auto observation=std::make_shared<ClusterInfo>();
  if (!front.measureUnseenWall(q,*observation)) return;
  observation->best_vp_=q;
  observation->contour_approach_=issued;
  const Vector3f tangent=q-issued[issued.size()-2];
  observation->best_vp_yaw_=std::atan2(tangent.y(),tangent.x());
  observation->close_surface_sample_=true;
  observation->displaced_issued_goal_=true;
  rememberObservation(observation);
  ROS_INFO("Saved unflown first observation at %.2f %.2f %.2f with %zu signed wall samples",
      q.x(),q.y(),q.z(),observation->contour_support_.size());
}

void FastExplorationManager::rememberObservation(const ClusterInfo::Ptr &observation) {
  auto &front=*frontier_manager_ptr_;
  const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
  const auto &goal=observation->best_vp_;
  auto existing=std::find_if(deferred_observations_.begin(),deferred_observations_.end(),[&](const auto &old) {
    vector<Vector3f> connection;
    return (goal-old->best_vp_).norm()<2.f*radius && front.connectObserved(goal,old->best_vp_,0.,connection);
  });
  if (existing!=deferred_observations_.end()) {
    if ((*existing)->displaced_issued_goal_ && !observation->displaced_issued_goal_) return;
    deferred_observations_.erase(existing);
  }
  observation->last_viewpoint_attempt_=-1.e9;
  deferred_observations_.push_back(observation);
  if (deferred_observations_.size()>size_t(std::max(1,ep_->local_viewpoint_num_))) {
    const auto sampled=std::find_if(deferred_observations_.begin(),deferred_observations_.end(),
        [](const auto &old){return !old->displaced_issued_goal_;});
    if (sampled!=deferred_observations_.end()) deferred_observations_.erase(sampled);
  }
}

void FastExplorationManager::appendDeferredObservation(const Vector3f &body,vector<TopoNode::Ptr> &views) {
  auto &front=*frontier_manager_ptr_;
  if (!front.surface_continuity_) return;
  const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
  const double now=epic_ros::Time::now().toSec();
  // During a bend, a single new scan can contain only the view through the
  // doorway. Keep the actual unflown wall arc available to this ATSP solve.
  if (published_wall_observation_ && !published_wall_observation_->exterior_pass_ && published_first_leg_ && now<departure_deadline_ &&
      published_first_route_.size()>1 && distanceToPolyline(body,published_first_route_)<2.f*radius) {
    const auto &goal=published_first_leg_->second;
    auto route=published_first_route_;
    if ((goal-body).norm()>2.f*radius && std::abs(goal.z()-body.z())<radius &&
        !front.visitedStance(goal) && front.hasLateralSurveyGain(goal,true) &&
        std::none_of(views.begin(),views.end(),[&](const auto &v){return (v->center_-goal).norm()<radius;}) &&
        reanchorContour(route,body,true) && front.certifyFlightGuide(route) &&
        std::all_of(route.begin(),route.end(),[&](const auto &p){return distanceToPolyline(p,published_first_route_)<2.f*radius;})) {
      auto wall=std::make_shared<ClusterInfo>(*published_wall_observation_);
      wall->contour_approach_=route;wall->contour_preview_=published_preview_;
      auto node=std::make_shared<TopoNode>();
      node->center_=goal;node->is_viewpoint_=true;
      const Vector3f edge=route.back()-route[route.size()-2];
      node->yaw_=std::atan2(edge.y(),edge.x());
      front.observations_[node.get()]=wall;
      views.push_back(node);
    }
  }
  bool productive=false;
  for (const auto &view:views) {
    const auto it=front.observations_.find(view.get());
    if (it==front.observations_.end() || std::abs(view->center_.z()-body.z())>radius ||
        front.visitedStance(view->center_) || front.unseenSupport(*it->second).empty()) continue;
    // A raw low stance can still be disconnected. Only a measured approach
    // can displace an earlier, productive pass before the graph search runs.
    if (it->second->contour_approach_.size()<2 ||
        (!it->second->close_surface_sample_ && it->second->contour_support_.empty())) continue;
    productive=true;
    auto saved=std::make_shared<ClusterInfo>(*it->second);
    saved->best_vp_=view->center_;
    saved->best_vp_yaw_=view->yaw_;
    rememberObservation(saved);
  }
  const bool entering=published_first_leg_ && published_sweep_entry_ && now<departure_deadline_;
  if ((productive && !entering) || deferred_observations_.empty()) return;

  ClusterInfo::Ptr candidate;
  float nearest=1.5f*front.sensorUpdateRange();
  for (auto it=deferred_observations_.begin();it!=deferred_observations_.end();) {
    const auto &obs=**it;
    const auto &q=obs.best_vp_;
    const bool issued=entering && (q-published_first_leg_->second).norm()<.01f &&
        (!obs.visible_cells_.empty() || std::any_of(obs.contour_support_.begin(),obs.contour_support_.end(),
            [](const auto &s){return s.kind==ObservationSample::Kind::WallFace;}));
    // Another aisle can enter the live set during a turn. Give the issued wall
    // observation a fresh connection in this solve while the vehicle enters.
    if (productive && !issued) {++it;continue;}
    if (front.visitedStance(q) || front.nearFlightPath(q,1.5f*radius,0.f,.35f*radius) ||
        front.observedCluster(obs) || (obs.contour_support_.empty() && front.surveyedFromAround(body,q)) ||
        front.unseenSupport(obs).empty() || !front.hasLateralSurveyGain(q,true) ||
        (!issued && !front.hasRevisitableSurface(obs,q))) {
      it=deferred_observations_.erase(it);
      continue;
    }
    const float distance=(q-body).norm();
    if (distance<nearest && std::abs(q.z()-body.z())<radius && now-obs.last_viewpoint_attempt_>=4. &&
        std::none_of(views.begin(),views.end(),[&](const auto &view){return (view->center_-q).norm()<radius;})) {
      candidate=*it;nearest=distance;
    }
    ++it;
  }
  if (!candidate) return;
  const auto goal=candidate->best_vp_;
  auto route=candidate->contour_approach_;
  if (!reanchorContour(route,body,true) && !front.connectObserved(body,goal,.025,route) &&
      !front.connectFlightHistory(body,goal,.035,route)) {
    if (now-last_observation_search_<2.) return;
    last_observation_search_=now;
    if (!front.connectDistantObserved(body,goal,route)) {
      candidate->last_viewpoint_attempt_=now;
      return;
    }
  }
  if (!front.certifyFlightGuide(route)) return;
  candidate->contour_approach_=route;
  auto view=std::make_shared<TopoNode>();
  view->center_=goal;view->is_viewpoint_=true;view->yaw_=candidate->best_vp_yaw_;
  auto observation=std::make_shared<ClusterInfo>(*candidate);
  observation->follows_local_outline_=false;
  if (observation->contour_entry_ && distanceToPolyline(*observation->contour_entry_,route)>radius)
    observation->contour_entry_.reset();
  if (observation->contour_preview_.size()>1 && !front.certifyFlightGuide(observation->contour_preview_))
    observation->contour_preview_.clear();
  front.observations_[view.get()]=observation;
  views.push_back(view);
  restored_observation_=goal;
  if (productive)
    ROS_INFO("Reconnected issued wall observation for the next tour before completing its entry");
  ROS_INFO("Restored measured observation %.1fm from body",nearest);
}

bool FastExplorationManager::skipsWallCorner(const vector<Vector3f> &preview,const vector<Vector3f> &approach,float radius) {
  if (preview.size()<2 || approach.size()<2) return false;
  const auto &wall=preview.front();
  const Vector2f span=(preview.back()-wall).head<2>();
  const float length=span.norm();
  if (length<2.f*radius) return false;
  const Vector2f axis=span/length,side(-axis.y(),axis.x());
  const Vector2f next=(approach.back()-wall).head<2>();
  if (next.dot(axis)<2.f*radius || next.dot(axis)>length+6.f*radius || std::abs(next.dot(side))>radius ||
      std::abs((approach.front()-wall).head<2>().dot(side))<2.f*radius ||
      std::abs(approach.front().z()-wall.z())>radius || std::abs(approach.back().z()-wall.z())>radius ||
      distanceToPolyline(wall,approach)<2.f*radius) return false;
  float last=0.f;
  for (const auto &p:preview) {
    const Vector2f delta=(p-wall).head<2>();
    const float along=delta.dot(axis);
    if (std::abs(p.z()-wall.z())>radius || std::abs(delta.dot(side))>radius || along<last-.25f*radius) return false;
    last=along;
  }
  return true;
}

bool FastExplorationManager::retracesEntryToWall(const vector<Vector3f> &approach,const Vector3f &velocity,
                                                 const Vector3f &entry,const Vector2f &inward,float radius) {
  if (approach.size()<3 || velocity.head<2>().norm()<1.f) return false;
  const auto &body=approach.front();
  const Vector2f offset=(body-entry).head<2>(),motion=velocity.head<2>().normalized();
  const float depth=offset.dot(inward);
  if (depth<radius || depth>6.f*radius || (offset-depth*inward).norm()>2.5f*radius ||
      motion.dot(inward)<.5f || std::abs(body.z()-entry.z())>radius) return false;
  const Vector2f goal=(approach.back()-body).head<2>();
  if ((approach.back()-entry).head<2>().dot(inward)<radius || std::abs(approach.back().z()-entry.z())>radius ||
      goal.norm()<2.f*radius || goal.dot(motion)<-.35f*goal.norm()) return false;
  for (size_t i=1;i+1<approach.size();++i) {
    const Vector2f step=(approach[i]-body).head<2>();
    if (step.norm()<radius) continue;
    return step.dot(motion)<-.6f*step.norm() && step.dot(inward)<-.6f*radius;
  }
  return false;
}

void FastExplorationManager::preservePublishedCorner(vector<Eigen::Vector3f> &approach,const Eigen::Vector3f &velocity) {
  if (!opening_guide_.empty() || !published_first_leg_ || approach.size()<2 ||
      published_first_route_.size()<2) return;
  const Vector3f body=approach.front(),goal=published_first_leg_->second;
  auto &front=*frontier_manager_ptr_;
  const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
  const auto &entry=issued_wall_entry_ ? issued_wall_entry_ : entered_turn_;
  if (entry && selected_observation_ && !selected_observation_->contour_support_.empty() &&
      !selected_observation_->exterior_pass_ && retracesEntryToWall(approach,velocity,entry->first,entry->second,radius)) {
    // Rejoin the same selected wall without commanding a return through the
    // jamb. Preserve the remainder of the ordered approach and its preview.
    const Vector2f motion=velocity.head<2>().normalized();
    for (size_t i=2;i<approach.size();++i) {
      const Vector2f step=(approach[i]-body).head<2>();
      if (step.norm()<radius || step.dot(motion)<-.35f*step.norm()) continue;
      vector<Vector3f> joined{body};
      joined.insert(joined.end(),approach.begin()+i,approach.end());
      if (guideSupported(joined) && front.certifyFlightGuide(joined)) {
        approach.swap(joined);
        ROS_INFO("Rejoined the selected wall without retracing the measured entrance");
        break;
      }
    }
  }
  // An advancing viewpoint on the same wall must keep the unflown approach
  // to that wall. A direct chord can otherwise bypass its first several metres.
  const bool measured=published_wall_observation_ ||
      (issued_wall_entry_ && epic_ros::Time::now().toSec()<issued_wall_entry_until_);
  if (measured && skipsWallCorner(published_preview_,approach,radius) && !front.flownObservationStance(goal)) {
    auto contour=published_first_route_;
    if (reanchorContour(contour,body,true)) {
      const Vector3f next=approach.back();
      const Vector2f axis=(published_preview_.back()-goal).head<2>().normalized();
      const float limit=(next-goal).head<2>().dot(axis);
      for (size_t i=1;i<published_preview_.size();++i) {
        const auto &p=published_preview_[i];
        if ((p-goal).head<2>().dot(axis)>=limit-.1f*radius) break;
        contour.push_back(p);
      }
      contour.push_back(next);
      if (guideSupported(contour) && front.certifyFlightGuide(contour)) {
        approach.swap(contour);
        ROS_INFO("Advanced first viewpoint along the unflown wall corner");
        return;
      }
    }
  }
  const auto incoming=front.recentFlightDirection(body);
  if (!incoming || !published_sweep_entry_ || (goal-body).norm()<=radius || front.visitedStance(goal) ||
      enteredSweep(body,*published_sweep_entry_,goal,radius)) return;
  // Both the superseded and the replacement guide are evaluated in the
  // same incoming frame. Preserve an actual near right opening only when
  // the replacement bypasses it or turns back through its entrance.
  auto committed=published_first_route_;
  if (!reanchorContour(committed,body,true) ||
      front.surveyDeparture(body,*incoming,committed,false,true)!=FrontierManager::Departure::RightOpening) return;
  const auto proposed=front.surveyDeparture(body,*incoming,approach,false,true);
  if (proposed!=FrontierManager::Departure::RightOpening &&
      (proposed==FrontierManager::Departure::Backwards || bypassesRightTurn(body,velocity,goal,approach,radius)))
    restorePublishedObservation(approach,std::move(committed),published_preview_);
}

bool FastExplorationManager::restorePublishedObservation(vector<Vector3f> &approach,vector<Vector3f> committed,
                                                         vector<Vector3f> preview) {
  const Vector3f body=approach.front(),old=committed.back(),remaining=old-body;
  // A validated ATSP first edge already accounted for wall continuity.
  // Handoff may smooth or extend it, but a previous preview cannot replace its goal.
  if (selected_observation_ && ed_->global_tour_.size()>1 &&
      (old-ed_->global_tour_[1]).squaredNorm()>.01f*.01f) return false;
  const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
  if (!reanchorContour(committed,body) || !frontier_manager_ptr_->certifyFlightGuide(committed)) return false;
  if (selected_observation_ && (old-approach.back()).norm()>3.f*radius &&
      std::abs(approach.back().z()-body.z())<radius &&
      !frontier_manager_ptr_->nearFlightPath(approach.back(),2.f*radius,0.f,.35f*radius) &&
      frontier_manager_ptr_->certifyFlightGuide(approach)) {
    auto saved=std::make_shared<ClusterInfo>(*selected_observation_);
    saved->best_vp_=approach.back();
    const Vector3f tangent=approach.back()-approach[approach.size()-2];
    saved->best_vp_yaw_=std::atan2(tangent.y(),tangent.x());
    saved->contour_approach_=approach;saved->contour_preview_=active_contour_preview_;
    rememberObservation(saved);
  }
  approach=std::move(committed);
  active_contour_approach_=approach;
  active_contour_preview_=std::move(preview);
  active_sweep_entry_=published_sweep_entry_;
  frontier_manager_ptr_->perimeter_target_.reset();
  ed_->global_tour_={body,old};
  selection_reason_="Restore the previous observation";
  const float yaw=std::atan2(remaining.y(),remaining.x());
  ed_->global_tour_yaws_={planner_manager_->local_data_.curr_yaw_,yaw};
  planner_manager_->local_data_.end_yaw_=yaw;
  updateGoalNode();
  return true;
}

// A later tour edge or measured preview may expose a premature U-turn only
// after the local guide is assembled. Finish the further, certified pass if
// the replacement turns left and runs all the way back past the aircraft.
bool FastExplorationManager::finishUnflownPass(vector<Vector3f> &path,size_t first_goal,const Vector3f &velocity) {
  if (!published_first_leg_ || published_first_route_.size()<2 ||
      !first_goal || first_goal>=path.size()) return false;
  const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
  auto &front=*frontier_manager_ptr_;
  const auto motion=front.recentFlightDirection(path.front());
  if (motion && !published_sweep_entry_ && !active_sweep_entry_) {
    const Vector3f start=path.front();
    auto previous=publishedGuide();
    auto through=forwardThroughGoal(previous,path,first_goal,velocity,radius,std::max(.1,ep_->a_avg_));
    if (!through.empty() && guideSupported(through) && front.certifyFlightGuide(through)) {
      path=std::move(through);
      ROS_INFO("Continued through selected observation on checked forward guide");
      return true;
    }
    if (reanchorContour(previous,start,true)) {
      // SUPER has already received this preview. Keep the reachable unflown
      // end if a transient shorter sample would brake or reverse along it.
      size_t far=0;
      float lead=0.f;
      for (size_t i=1;i<previous.size();++i) {
        const Vector3f step=previous[i]-start;
        const float along=step.head<2>().dot(*motion);
        const float lateral=(step.head<2>()-along*(*motion)).norm();
        if (std::abs(step.z())>radius || along<lead-radius || lateral>2.f*radius+.15f*std::max(0.f,along)) break;
        if (along>lead) {lead=along;far=i;}
      }
      float advance=0.f;
      bool new_turn=false;
      for (const auto &q:path) {
        const Vector3f step=q-start;
        const float along=step.head<2>().dot(*motion);
        advance=std::max(advance,along);
        if (std::abs(step.z())<radius && !front.visitedStance(q) &&
            along>=0.f && (step.head<2>()-along*(*motion)).norm()>2.f*radius) new_turn=true;
      }
      if (!new_turn && lead>3.f*radius && advance+2.f*radius<lead && !front.visitedStance(previous[far])) {
        previous.resize(far+1);
        const Vector3f goal=previous.back();
        vector<Vector3f> approach(path.begin(),path.begin()+first_goal+1);
        if (restorePublishedObservation(approach,std::move(previous),{goal})) {
          path.swap(approach);opening_guide_.clear();
          ROS_INFO("Retained %.2fm of certified unflown forward guide",lead);
          return true;
        }
      }
    }
  }
  if (!opening_guide_.empty() || first_goal+1>=path.size()) return false;
  const Vector3f body=path.front(),remaining=published_first_leg_->second-body;
  const float length=remaining.head<2>().norm(),speed=velocity.head<2>().norm();
  if (length<4.f*radius || speed<1.f || std::abs(remaining.z())>radius ||
      velocity.head<2>().dot(remaining.head<2>())<.75f*speed*length) return false;
  const Vector2f heading=remaining.head<2>()/length,left(-heading.y(),heading.x());
  const Vector3f tip=path.back()-body;
  if (tip.head<2>().dot(heading)>-radius || tip.head<2>().dot(left)<2.f*radius) return false;
  float farthest=0.f;
  for (const auto &q:path) {
    if (std::abs(q.z()-body.z())>radius) return false;
    farthest=std::max(farthest,(q-body).head<2>().dot(heading));
  }
  if (farthest<2.f*radius || length-farthest<2.f*radius) return false;
  bool fresh=false;
  for (const auto &q:published_first_route_) {
    const Vector2f delta=(q-body).head<2>();
    if (delta.dot(heading)>farthest+radius && std::abs(delta.dot(left))<radius &&
        !frontier_manager_ptr_->visitedStance(q)) {fresh=true;break;}
  }
  if (!fresh) return false;
  vector<Vector3f> approach(path.begin(),path.begin()+first_goal+1);
  if (!restorePublishedObservation(approach,published_first_route_,published_preview_)) return false;
  const auto &preview=active_contour_preview_;
  if (preview.size()>1 && (approach.back()-preview.front()).norm()<.01f &&
      guideSupported(preview) && frontier_manager_ptr_->certifyFlightGuide(preview))
    approach.insert(approach.end(),preview.begin()+1,preview.end());
  path.swap(approach);
  return true;
}

// A first viewpoint can sit just before a right bend, with only a short
// measured segment visible beyond it. Remember the observed direction in
// the map frame while the next scan resolves the interior.
void FastExplorationManager::rememberShortRightTurn(const vector<Vector3f> &approach,const vector<Vector3f> &preview) {
  if (approach.size()<2) return;
  const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
  const auto remember=[&](size_t pivot,const Vector3f &lip) {
    const auto &corner=approach[pivot];
    const Vector2f onward=(lip-corner).head<2>();
    if (onward.norm()<radius || onward.norm()>3.f*radius || std::abs(lip.z()-corner.z())>.45f*radius) return false;
    size_t i=pivot-1;
    while (i && (corner-approach[i]).head<2>().norm()<3.f*radius) --i;
    const Vector2f incoming=(corner-approach[i]).head<2>();
    if (incoming.norm()<2.f*radius || std::abs(corner.z()-approach[i].z())>.5f*radius) return false;
    const Vector2f right(incoming.y(),-incoming.x());
    if (onward.dot(right)<.80f*onward.norm()*incoming.norm()) return false;
    const double now=epic_ros::Time::now().toSec();
    if (!short_right_turn_ || now>=short_turn_until_ || (short_right_turn_->first-lip).norm()>=2.f*radius) {
      short_right_turn_=std::make_pair(lip,onward.normalized());
      short_turn_until_=now+3.+std::min(6.f,(corner-approach.front()).norm()/2.f);
    }
    return true;
  };
  if (preview.size()>1) {
    remember(approach.size()-1,preview.back());
    return;
  }
  // #1 can already be a short distance inside the turn. A fresh scan sees
  // the same corner regardless of which side of the TSP vertex it falls on.
  const auto &lip=approach.back();
  for (size_t i=approach.size()-2;i>0;--i) {
    if ((lip-approach[i]).norm()>3.f*radius) break;
    if (remember(i,lip)) return;
  }
}

// A viewpoint can be across the first interior wall, followed by a preview
// that leaves the opening. Only the ordered approach to #1 defines entry.
bool FastExplorationManager::reachesInteriorObservation(const vector<Vector3f> &route,const Vector3f &goal,
    const Vector3f &lip,const Vector2f &inward,float radius) {
  const auto depth=[&](const Vector3f &p){return (p-lip).head<2>().dot(inward);};
  if (route.size()<2 || depth(goal)<.5f*radius || std::abs(goal.z()-lip.z())>radius) return false;
  const auto at_door=[&](const Vector3f &p) {
    const Vector2f d=(p-lip).head<2>();
    return (d-depth(p)*inward).norm()<2.5f*radius && std::abs(p.z()-lip.z())<radius;
  };
  bool entered=depth(route.front())>=0.f && at_door(route.front());
  for (size_t i=1;i<route.size();++i) {
    const auto &a=route[i-1],&b=route[i];
    const float da=depth(a),db=depth(b);
    if (da<=0.f && db>0.f) entered=at_door(a+(-da/(db-da))*(b-a));
    if (db<-.25f*radius || std::abs(b.z()-lip.z())>radius) entered=false;
    if ((b-goal).norm()<.1f) return entered;
  }
  return false;
}

bool FastExplorationManager::selectedWallInsideTurn(const vector<Vector3f> &route,const Vector3f &lip,
    const Vector2f &inward,float radius) const {
  if (!selected_observation_ || selected_observation_->contour_support_.empty() ||
      selected_observation_->exterior_pass_ || ed_->global_tour_.size()<2 || route.size()<2) return false;
  const auto &goal=ed_->global_tour_[1];
  if (reachesInteriorObservation(route,goal,lip,inward,radius)) return true;
  // A fresh connection sometimes rejoins the old doorway from behind. Only
  // accept the alternate entrance test if a direct measured connection to
  // this same observation also crosses the opening on the interior side.
  const vector<Vector3f> direct{route.front(),goal};
  return reachesInteriorObservation(direct,goal,lip,inward,radius) &&
      frontier_manager_ptr_->certifyFlightGuide(direct);
}

// If the new handoff abandons that turn, use only the newly measured,
// collision-checked continuation. Release the commitment as soon as the
// vehicle crosses it or a real replacement continues into the same space.
bool FastExplorationManager::continueShortRightTurn(vector<Vector3f> &route) {
  if (!short_right_turn_ || route.size()<2) return false;
  const auto [lip,axis]=*short_right_turn_;
  const float radius=planner_manager_->topo_graph_->bubble_min_radius_,cell=frontier_manager_ptr_->getCellSize();
  const Vector3f body=route.front();
  const Vector2f offset=(body-lip).head<2>();
  if (epic_ros::Time::now().toSec()>short_turn_until_ || offset.dot(axis)>2.75f*radius ||
      std::abs(body.z()-lip.z())>radius) {short_right_turn_.reset();return false;}
  if (offset.norm()>4.f*radius) return false;
  if (selectedWallInsideTurn(route,lip,axis,radius)) {
    entered_turn_=short_right_turn_;
    short_right_turn_.reset();
    ROS_INFO("Forwarding selected interior wall through the measured opening");
    return false;
  }
  const Vector2f end=(route.back()-lip).head<2>();
  const float end_depth=end.dot(axis);
  // A short true continuation can be the next wall turn. It has already
  // crossed the lip, even when its endpoint is closer than our preview.
  if (end_depth>.25f*cell && (end-end_depth*axis).norm()<2.f*radius) return false;
  for (const auto &q:route) {
    const Vector2f delta=(q-lip).head<2>();
    const float depth=delta.dot(axis);
    if (depth>1.75f*radius && (delta-depth*axis).norm()<2.5f*radius) return false;
  }
  auto &front=*frontier_manager_ptr_;
  // A transient entry preview must not replace a complete, advancing wall
  // guide. Keep a measured doorway commitment protected by the normal guard.
  if (right_hand_priority_ && !issued_wall_entry_ && selected_observation_ &&
      selected_observation_->follows_local_outline_ &&
      selected_observation_->fresh_contour_length_>=2.f*radius) {
    const auto incoming=front.recentFlightDirection(body);
    const auto departure=std::find_if(route.begin()+1,route.end(),[&](const Vector3f &point) {
      return (point-body).head<2>().norm()>=radius;
    });
    if (incoming && departure!=route.end() &&
        (*departure-body).head<2>().dot(*incoming)>0.f &&
        (route.back()-body).head<2>().norm()>4.f*radius &&
        guideSupported(route) && front.certifyFlightGuide(route)) {
      ROS_INFO("Kept the advancing wall guide past a transient turn preview");
      return false;
    }
  }
  if (front.flownObservationStance(lip)) return false;
  Vector3f tip=lip;
  for (float depth=cell;depth<=4.f*radius;depth+=cell) {
    Vector3f q=lip;q.head<2>()+=axis*depth;
    if (!front.certifyFlightGuide({lip,q})) break;
    if (!front.visitedStance(q)) tip=q;
  }
  vector<Vector3f> through;
  if ((tip-lip).norm()>=1.5f*radius && (tip-body).head<2>().dot(axis)>=.5f*radius) {
    through={body,tip};
    if (!front.certifyFlightGuide(through)) {
      through={body,lip,tip};
      if (offset.dot(axis)>0.f || !front.certifyFlightGuide(through)) through.clear();
    }
  }
  if (through.empty()) {
    // A wall just inside the turn can cut the original tangent short.
    // Use the connected observed passage only if it enters the same opening.
    if (!front.surface_continuity_ || !front.observedBoundaryGuide(body,through,&axis,nullptr,2.f*radius,nullptr,
            front.currentSurveyBand(body)) || !front.certifyFlightGuide(through) ||
        front.flownObservationStance(through.back())) return false;
    bool crossed=false;
    for (const auto &q:through) {
      const Vector2f delta=(q-lip).head<2>();
      const float depth=delta.dot(axis);
      if (depth<offset.dot(axis)-radius || std::abs(q.z()-lip.z())>radius) return false;
      if (depth<1.5f*radius) continue;
      crossed=(delta-depth*axis).norm()<2.f*radius;
      break;
    }
    if (!crossed) return false;
  }
  if (!guideSupported(through)) return false;
  route=std::move(through);
  entered_turn_=short_right_turn_;
  return true;
}

// A new guide can pass the end of a measured turn while the vehicle is still
// on approach. Compare the first crossing with the first lateral departure.
bool FastExplorationManager::passesBesideEntry(const vector<Vector3f> &route,const Vector3f &lip,
                                               const Vector2f &inward,float radius) {
  if (route.size()<2) return false;
  const Vector2f offset=(route.front()-lip).head<2>(),side(-inward.y(),inward.x());
  const float plane=-.25f*radius, width=2.5f*radius;
  if (offset.norm()>4.f*radius || offset.dot(inward)>-.5f*radius ||
      std::abs(offset.dot(side))>=width || std::abs(route.front().z()-lip.z())>radius) return false;
  for (size_t i=1;i<route.size();++i) {
    if (std::abs(route[i].z()-lip.z())>radius) return false;
    const Vector2f a=(route[i-1]-lip).head<2>(),b=(route[i]-lip).head<2>();
    const float da=a.dot(inward),db=b.dot(inward);
    if (da<plane && db>=plane) {
      const Vector2f crossing=a+(plane-da)/(db-da)*(b-a);
      if (std::abs(crossing.dot(side))<=width) return false;
    }
    if (std::abs(b.dot(side))>width) return true;
  }
  return false;
}

bool FastExplorationManager::exitsThroughEntry(const vector<Vector3f> &route,const Vector3f &entry,
                                              const Vector2f &inward,float radius) {
  if (route.size()<2 || (route.front()-entry).head<2>().dot(inward)<.5f*radius) return false;
  for (size_t i=1;i<route.size();++i) {
    const Vector2f a=(route[i-1]-entry).head<2>(),b=(route[i]-entry).head<2>();
    const float first=a.dot(inward),last=b.dot(inward);
    if (first<0.f || last>=0.f) continue;
    const float t=first/(first-last);
    const float z=(1.f-t)*route[i-1].z()+t*route[i].z();
    if (std::abs(z-entry.z())<=radius && (a+t*(b-a)).norm()<=2.5f*radius) return true;
  }
  return false;
}

// If a subsequent solve republishes #1 farther along a straight, the
// original corner is no longer in its polyline. A full reversal beyond
// that straight's start still abandons an outstanding observation.
static bool flipsStraightPass(const vector<Vector3f> &issued,const vector<Vector3f> &next,float radius) {
  if (issued.size()<2 || next.size()<2) return false;
  const auto &start=issued.front();
  const Vector2f span=(issued.back()-start).head<2>();
  const float length=span.norm();
  if (length<4.f*radius) return false;
  const Vector2f axis=span/length,side(-axis.y(),axis.x());
  const auto along=[&](const Vector3f &p){return (p-start).head<2>().dot(axis);};
  const auto on_strip=[&](const Vector3f &p,float width) {
    return std::abs(p.z()-start.z())<radius && std::abs((p-start).head<2>().dot(side))<width;
  };
  const float body=along(next.front()),end=along(next.back());
  if (body<=-3.f*radius || body>=length-3.f*radius || end>=-radius || end>=body-2.f*radius) return false;
  return std::all_of(issued.begin(),issued.end(),[&](const Vector3f &p){return on_strip(p,1.5f*radius);}) &&
      std::all_of(next.begin(),next.end(),[&](const Vector3f &p){return on_strip(p,2.5f*radius) && along(p)<body+radius;});
}

// Detect a replacement that goes back through the same fork towards its
// opposite arm while the published first goal is still ahead on the right.
bool FastExplorationManager::backtracksRightTurn(const vector<Vector3f> &issued,const vector<Vector3f> &replacement,float radius) {
  if (issued.size()<3 || replacement.size()<2) return false;
  const auto &tip=issued.back(),&body=replacement.front();
  size_t pivot=issued.size()-2;
  while (pivot && (tip-issued[pivot]).head<2>().norm()<2.f*radius) --pivot;
  const Vector2f tail=(tip-issued[pivot]).head<2>();
  if (tail.norm()<2.f*radius) return false;
  const Vector2f axis=tail.normalized();
  while (pivot) {
    const Vector2f step=(issued[pivot]-issued[pivot-1]).head<2>();
    if (step.dot(axis)<.8f*step.norm()) break;
    --pivot;
  }
  const Vector2f approach=(issued[pivot]-issued.front()).head<2>();
  const Vector2f offset=(body-issued[pivot]).head<2>();
  if (approach.norm()<1.25f*radius || axis.dot(Vector2f(approach.y(),-approach.x()))<.7f*approach.norm() ||
      std::abs(tip.z()-issued[pivot].z())>radius || std::abs(body.z()-tip.z())>radius ||
      offset.dot(axis)<.5f*radius || (offset-offset.dot(axis)*axis).norm()>1.5f*radius ||
      (tip-body).head<2>().dot(axis)<2.f*radius) return false;
  for (const auto &point:replacement) {
    const Vector2f step=(point-body).head<2>();
    if (step.norm()<radius) continue;
    return step.dot(axis)<-.8f*step.norm() && (replacement.back()-issued[pivot]).head<2>().dot(axis)<-radius;
  }
  return false;
}

// A distant transfer through a previously flown passage can cut the inside
// wall out of the current sweep. Keep only a still-executable wall observation
// that the replacement misses. A continuation on the same wall remains free
// to move #1 forward as new scans arrive.
template<class CanFollow>
static bool cutsActiveWallPass(FrontierManager &front,const vector<Vector3f> &issued,
                               const vector<Vector3f> &preview,const vector<Vector3f> &next,float radius,
                               bool measured_outline,const CanFollow &can_follow) {
  if (issued.size()<2 || preview.size()<2 || next.size()<2) return false;
  const auto &body=next.front();
  const float transfer=front.sensorUpdateRange();
  if ((next.back()-body).head<2>().norm()<.75f*transfer ||
      distanceToPolyline(body,issued)>2.f*radius) return false;
  Vector3f target=issued.back();
  if (distanceToPolyline(target,next)<2.f*radius) target=preview.back();
  if (distanceToPolyline(target,next)<2.f*radius || std::abs(target.z()-body.z())>radius ||
      (target-body).norm()<2.f*radius || front.flownObservationStance(target) ||
      (!measured_outline && !front.followsRightWall(issued) && !front.followsRightWall(preview))) return false;
  float along=0.f,previously_flown=0.f;
  for (size_t i=1;i<next.size();++i) {
    const float step=(next[i]-next[i-1]).norm();along+=step;
    if (along<3.f*radius || std::abs(next[i].z()-body.z())>radius ||
        !front.nearFlightPath(next[i],1.5f*radius,.5f*transfer,.35f*radius,radius)) {
      previously_flown=0.f;continue;
    }
    previously_flown+=step;
    // Crossing an already-flown doorway can be shorter than following the
    // old track. A close crossing is sufficient for a measured wall outline.
    const bool crosses=measured_outline && (next.back()-next[i]).head<2>().norm()>2.f*radius &&
        front.nearFlightPath(next[i],radius,.5f*transfer,.35f*radius,radius);
    if (previously_flown<1.5f*radius && !crosses) continue;
    if (can_follow(target)) return true;
    vector<Vector3f> link;
    return front.connectObserved(body,target,.018,link) && front.certifyFlightGuide(link) &&
        std::all_of(link.begin(),link.end(),[&](const auto &q) {
          return std::min(distanceToPolyline(q,issued),distanceToPolyline(q,preview))<2.f*radius;
        });
  }
  return false;
}

// A short preview through a measured turn has already been sent to SUPER.
// Recheck it against the current scan before letting a subsequent solve brake
// at the entry or turn back across it. Release on progress or on a new obstacle.
bool FastExplorationManager::retractsMeasuredEntry(const vector<Vector3f> &replacement) {
  if (!published_first_leg_ || replacement.size()<2) return false;
  const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
  const double now=epic_ros::Time::now().toSec();
  const auto &body=replacement.front();
  auto &front=*frontier_manager_ptr_;
  // The collision-checked approach turned at the jamb. The straight chord
  // from the current body to the wall can cut that jamb. Test the remaining
  // issued polyline before discarding a still-executable turn.
  const auto can_follow_issued=[&](const Vector3f &tip) {
    if (front.certifyFlightGuide({body,tip})) return true;
    vector<Vector3f> issued=published_first_route_;
    if (published_preview_.size()>1) issued.insert(issued.end(),published_preview_.begin()+1,published_preview_.end());
    const auto end=std::find_if(issued.begin(),issued.end(),[&](const auto &p){return (p-tip).norm()<.2f*radius;});
    if (end==issued.end()) return false;
    const auto nearest=std::min_element(issued.begin(),end+1,[&](const auto &a,const auto &b){
      return (a-body).squaredNorm()<(b-body).squaredNorm();
    });
    if ((*nearest-body).norm()>2.f*radius) return false;
    vector<Vector3f> link{body};
    link.insert(link.end(),nearest+1,end+1);
    if (front.certifyFlightGuide(link)) return true;
    link.insert(link.begin()+1,*nearest);
    return front.certifyFlightGuide(link);
  };
  if (right_hand_priority_ && published_right_hand_) {
    const auto &tip=published_first_leg_->second;
    const float distance=(tip-body).norm();
    if (distance<right_hand_best_distance_-.15f) {
      right_hand_best_distance_=distance;
      right_hand_progress_time_=now;
    }
    if (distance>.5f*radius && now<right_hand_deadline_ && now-right_hand_progress_time_<2.5 &&
        distanceToPolyline(tip,replacement)>radius && !front.flownObservationStance(tip) &&
        can_follow_issued(tip)) {
      ROS_INFO("Right-hand priority: keeping certified issued wall at %.2f %.2f",tip.x(),tip.y());
      return true;
    }
  }
  if (short_right_turn_ && now<short_turn_until_) {
    const auto &[lip,inward]=*short_right_turn_;
    const bool on_issued_guide=distanceToPolyline(lip,published_first_route_)<.5f*radius ||
        distanceToPolyline(lip,published_preview_)<.5f*radius;
    if (on_issued_guide && passesBesideEntry(replacement,lip,inward,radius) &&
        !frontier_manager_ptr_->flownObservationStance(lip) &&
        can_follow_issued(lip)) {
      ROS_INFO("Keeping issued right turn while approaching the opening");
      return true;
    }
  }
  if (issued_wall_entry_ && now<issued_wall_entry_until_) {
    const auto &[door,inward]=*issued_wall_entry_;
    const auto &wall=published_first_leg_->second;
    const float reach=(wall-door).head<2>().dot(inward);
    Vector3f gate=door;
    gate.head<2>()+=inward*std::clamp(reach,0.f,radius);
    const bool bypass=passesBesideEntry(replacement,gate,inward,radius);
    // A rolling preview may leave only *after* observing this wall. Reject
    // an early exit through the same entry, never the correct later exit.
    const bool early_exit=(wall-body).norm()>1.5f*radius && distanceToPolyline(wall,replacement)>2.f*radius &&
        exitsThroughEntry(replacement,door,inward,radius);
    if ((bypass || early_exit) && !front.flownObservationStance(wall) && can_follow_issued(wall)) {
      ROS_INFO("Finishing measured interior wall before exiting its entry");
      return true;
    }
  }
  const bool native_wall=published_wall_observation_ && !published_wall_observation_->exterior_pass_ &&
      published_wall_observation_->fresh_contour_length_>=2.f*radius && published_preview_.size()>1;
  // Reaching the first stance and flying its certified wall preview take
  // different amounts of time. The latter deadline stays tied to the issued
  // goal, so repeated solves cannot lock the aircraft on a spent wall.
  const double onward=native_wall ? std::min(3.,(published_preview_.back()-published_preview_.front()).norm()/
      std::max(1.,.5*ep_->v_max_)) : 0.;
  if (now<departure_deadline_+onward &&
      cutsActiveWallPass(front,published_first_route_,published_preview_,replacement,radius,native_wall,can_follow_issued)) {
    ROS_INFO("Kept an unflown wall before transferring through previously flown space");
    return true;
  }
  if (now>=departure_deadline_) return false;
  // The synthetic entry extension has already served its purpose once the
  // tour supplies a certified wall observation inside this very opening.
  if (!published_wall_observation_ && entered_turn_ &&
      selectedWallInsideTurn(replacement,entered_turn_->first,entered_turn_->second,radius)) return false;
  const auto &tip=published_first_leg_->second;
  const Vector2f to_tip=(tip-body).head<2>();
  // Compare the first metres actually commanded. The endpoint can lie in an
  // adjacent aisle even though reaching it starts by flying back down this one.
  if (to_tip.norm()>radius && std::abs(tip.z()-body.z())<radius &&
      !frontier_manager_ptr_->flownObservationStance(tip)) {
    const auto departure=std::find_if(replacement.begin()+1,replacement.end(),[&](const auto &p) {
      return (p-body).template head<2>().norm()>=1.5f*radius;
    });
    if (departure!=replacement.end() && std::abs(departure->z()-body.z())<radius) {
      const Vector2f step=(*departure-body).head<2>().normalized();
      const auto incoming=frontier_manager_ptr_->recentFlightDirection(body);
      const bool retraces=incoming && to_tip.dot(*incoming)>.6f*to_tip.norm() &&
          step.dot(*incoming)<-.65f && frontier_manager_ptr_->nearFlightPath(*departure,1.5f*radius,0.f,.35f*radius,radius);
      // With inertia the aircraft can overshoot the corner before the new
      // polynomial takes effect. Keep a reachable issued right-hand wall.
      const Vector2f entry=(tip-published_first_leg_->first).head<2>();
      const bool abandons_wall=published_wall_observation_ && entry.norm()>3.f*radius &&
          to_tip.dot(entry)>.6f*to_tip.norm()*entry.norm() && step.dot(entry)<-.35f*entry.norm() &&
          distanceToPolyline(tip,replacement)>2.f*radius;
      if ((retraces || abandons_wall) && frontier_manager_ptr_->certifyFlightGuide({body,tip})) return true;
    }
  }
  if ((selected_observation_ && selected_observation_->exterior_pass_)) return false;
  if (!published_first_route_.empty() &&
      (backtracksRightTurn(published_first_route_,replacement,radius) || flipsStraightPass(published_first_route_,replacement,radius)) &&
      !frontier_manager_ptr_->flownObservationStance(published_first_route_.back())) {
    const auto &tip=published_first_route_.back();
    if (frontier_manager_ptr_->certifyFlightGuide({replacement.front(),tip})) return true;
    // The aircraft may have drifted outside the straight while replanning.
    // Require a checked connector into the already issued route in that case.
    vector<Vector3f> join;
    if (frontier_manager_ptr_->connectObserved(replacement.front(),tip,.012,join) &&
        frontier_manager_ptr_->certifyFlightGuide(join) &&
        std::all_of(join.begin(),join.end(),[&](const Vector3f &p) {
          return distanceToPolyline(p,published_first_route_)<2.5f*radius;
        })) return true;
  }
  if (published_first_route_.size()>2) {
    auto &front=*frontier_manager_ptr_;
    const auto &tip=published_first_route_.back(),&body=replacement.front();
    const Vector2f start=(published_first_route_[1]-published_first_route_.front()).head<2>();
    const Vector2f tail=(tip-published_first_route_[published_first_route_.size()-2]).head<2>();
    const Vector2f remaining=(tip-body).head<2>(),proposed=(replacement.back()-body).head<2>();
    // Scanning a wall during the approach can remove its frontier from the
    // next solve. Finish the issued bend if that solve only sends SUPER
    // back into previously flown space on the opposite side of the turn.
    if (start.norm()>.2f && tail.norm()>.2f && remaining.norm()>3.f*radius && proposed.norm()>radius &&
        std::abs(start.dot(tail))<.65f*start.norm()*tail.norm() &&
        remaining.dot(tail)>.7f*remaining.norm()*tail.norm() &&
        proposed.dot(tail)<-.5f*proposed.norm()*tail.norm() &&
        std::abs(tip.z()-body.z())<radius && std::abs(replacement.back().z()-body.z())<radius &&
        distanceToPolyline(tip,replacement)>2.f*radius && !front.flownObservationStance(tip) &&
        front.flownObservationStance(replacement.back()) && front.followsRightWall(published_first_route_) &&
        front.certifyFlightGuide({body,tip})) return true;
  }
  if (!published_sweep_entry_ || published_preview_.size()<2) return false;
  const auto &entry=*published_sweep_entry_,&preview_tip=published_preview_.back();
  if ((entry-published_first_leg_->second).norm()>.25f*radius ||
      std::abs(preview_tip.z()-entry.z())>.5f*radius || std::abs(body.z()-entry.z())>radius) return false;
  const Vector2f span=(preview_tip-entry).head<2>();
  const float length=span.norm();
  if (length<1.5f*radius || length>3.5f*radius) return false;
  const Vector2f axis=span/length;
  const float required=std::min(length,2.5f*radius)-.35f*radius;
  const Vector2f offset=(body-entry).head<2>();
  if (offset.dot(axis)>=required || (offset-offset.dot(axis)*axis).norm()>3.f*radius) return false;
  for (const auto &q:replacement) {
    const Vector2f delta=(q-entry).head<2>();
    const float depth=delta.dot(axis);
    if (depth>=required && (delta-depth*axis).norm()<2.f*radius) return false;
    if (depth<offset.dot(axis)-radius) break;
  }
  const Vector3f start=offset.dot(axis)>0.f ? body : entry;
  return frontier_manager_ptr_->certifyFlightGuide({body,start,preview_tip});
}

// A second tour edge can immediately leave the shallow opening reached by
// #1. A fast local trajectory then rounds the entire excursion away. Keep
// an observed continuation through the opening while the next scan arrives.
bool FastExplorationManager::extendMeasuredTurn(vector<Vector3f> &path,size_t first_goal,const Vector3f &velocity) {
  if (!opening_guide_.empty() || !first_goal || first_goal>=path.size()) return false;
  const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
  const Vector2f motion=velocity.head<2>();
  if (motion.norm()<.5f) return false;
  const Vector2f forward=motion.normalized(),right(forward.y(),-forward.x());
  const Vector3f goal=path[first_goal];
  const Vector2f displacement=(goal-path.front()).head<2>();
  if (displacement.dot(right)<radius || displacement.dot(forward)<-.5f*radius) return false;
  size_t before=first_goal-1;
  while (before && (path[before]-goal).head<2>().norm()<1.35f*radius) --before;
  const Vector2f arrival=(goal-path[before]).head<2>();
  if (arrival.norm()<radius || arrival.normalized().dot(right)<.65f) return false;
  const Vector2f inward=arrival.normalized();
  bool exits=false;
  if (first_goal+1<path.size()) {
    for (size_t i=first_goal+1;i<path.size();++i) {
      const float depth=(path[i]-goal).head<2>().dot(inward);
      if (depth>=radius) {
        if (std::abs(path[i].z()-goal.z())<.5f*radius) active_sweep_entry_=goal;
        return false;
      }
      if (depth<-.65f*radius) {exits=true;break;}
    }
    if (!exits) return false;
  }
  Vector3f tip=goal;
  const float cell=frontier_manager_ptr_->getCellSize();
  for (float step=cell;step<=3.f*radius;step+=cell) {
    Vector3f next=goal;next.head<2>()+=step*inward;
    if (frontier_manager_ptr_->visitedStance(next) || !frontier_manager_ptr_->certifyFlightGuide({tip,next})) break;
    tip=next;
  }
  const bool extends=(tip-goal).norm()>=radius && frontier_manager_ptr_->certifyFlightGuide({goal,tip});
  if (!extends && !exits) return false;
  // Sparse first scans may only certify the lip. Do not forward a tour
  // edge back out of it while waiting for the next inward observation.
  path.resize(first_goal+1);
  if (extends) path.push_back(tip);
  active_sweep_entry_=goal;
  return true;
}

bool FastExplorationManager::retracesApproach(const vector<Eigen::Vector3f> &path,size_t first_goal) const {
  const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
  const auto sharesApproach=[&](const Eigen::Vector3f &point,const Eigen::Vector3f &direction) {
    for (size_t j=1;j<=first_goal;++j) {
      const Eigen::Vector3f leg=path[j]-path[j-1];
      const float size=leg.norm();
      if (size<.1f || direction.dot(leg)>-.65f*size) continue;
      const float along=(point-path[j-1]).dot(leg)/size;
      if (along<0.f || along>size) continue;
      const Eigen::Vector3f closest=path[j-1]+(along/size)*leg;
      const float separation=(closest-point).norm();
      if (separation>3.f*radius) continue;
      // Parallel lanes separated by a wall are distinct sweeps.
      if (separation<.05f || frontier_manager_ptr_->certifyFlightGuide({point,closest})) return true;
    }
    return false;
  };
  float reverse=0.f;
  for (size_t i=first_goal+1;i<path.size();++i) {
    const Eigen::Vector3f step=path[i]-path[i-1];
    const float length=step.norm();
    if (length<.01f) continue;
    const Eigen::Vector3f direction=step/length;
    const int probes=std::max(1,int(std::ceil(length/std::max(.1f,radius))));
    const float interval=length/probes;
    for (int k=0;k<probes;++k) {
      const Eigen::Vector3f midpoint=path[i-1]+(k+.5f)*interval*direction;
      reverse=sharesApproach(midpoint,direction) ? reverse+interval : 0.f;
      if (reverse>1.5f*radius) return true;
    }
  }
  return false;
}

// The end of a long hairpin preview can lie next to the vehicle while the
// selected viewpoint is still several metres ahead. Retain a fly-through
// tangent at that viewpoint until the vehicle enters its neighbourhood.
void FastExplorationManager::limitFoldedPreview(vector<Eigen::Vector3f> &path,size_t first_goal) const {
  if (first_goal==0 || first_goal+1>=path.size()) return;
  const float lead=1.5f*planner_manager_->topo_graph_->bubble_min_radius_;
  if ((path[first_goal]-path.front()).norm()<=lead || !retracesApproach(path,first_goal)) return;
  float left=lead;
  for (size_t i=first_goal+1;i<path.size();++i) {
    const Eigen::Vector3f edge=path[i]-path[i-1];
    const float length=edge.norm();
    if (length>=left && length>1.e-5f) {
      path[i]=path[i-1]+(left/length)*edge;
      path.resize(i+1);
      return;
    }
    left-=length;
  }
}

bool FastExplorationManager::enteredSweep(const Eigen::Vector3f &body,const Eigen::Vector3f &entry,
                                         const Eigen::Vector3f &goal,float tolerance) {
  const Eigen::Vector3f outgoing=goal-entry;
  if ((body-entry).norm()<=tolerance || (body-goal).norm()<=tolerance) return true;
  if (outgoing.squaredNorm()<.01f) return false;
  const float fraction=(body-entry).dot(outgoing)/outgoing.squaredNorm();
  return fraction>0.f && fraction<=1.f && (body-entry-fraction*outgoing).norm()<=tolerance;
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
  selection_reason_="Reachable observation";
  restored_observation_.reset();
  active_contour_approach_.clear();
  active_contour_preview_.clear();
  active_sweep_entry_.reset();
  selected_observation_.reset();
  if (entered_turn_) {
    const auto &[entry,inward]=*entered_turn_;
    const Vector3f delta=current_body_position-entry;
    if (delta.head<2>().dot(inward)<-2.f*planner_manager_->topo_graph_->bubble_min_radius_ ||
        std::abs(delta.z())>planner_manager_->topo_graph_->bubble_min_radius_ ||
        delta.head<2>().norm()>frontier_manager_ptr_->sensorUpdateRange()) entered_turn_.reset();
  }
  // A measured viewpoint approach can provide the first edge of an isolated odometry node.
  frontier_manager_ptr_->generateTSPViewpoints(current_body_position,viewpoints);
  const size_t sampled_views=viewpoints.size();
  const bool exterior_pending=frontier_manager_ptr_->exteriorPassPending();
  if (!exhausted_recovery_route_.empty()) {
    // The vehicle may coast during the confirmation interval. Re-evaluate
    // current observations before a cached tour or another recovery can move it.
    auto &front=*frontier_manager_ptr_;
    const bool new_observation=std::any_of(viewpoints.begin(),viewpoints.end(),[&](const auto &view) {
      const auto observation=front.observations_.find(view.get());
      return observation==front.observations_.end() ||
             (!front.flownObservationStance(view->center_) && !front.unseenSupport(*observation->second).empty());
    });
    if (!exterior_pending && frontier_empty_since_>=0. && !new_observation &&
        front.exhaustedAboveFlownFloor(exhausted_recovery_route_)) {
      front.publishSurveyViewpoints(viewpoints);
      ed_->global_tour_.clear();
      return frontierStatus(current_body_position,true);
    }
    exhausted_recovery_route_.clear();
    frontier_empty_since_=-1.;
  }
  const auto exterior=[&](const TopoNode::Ptr &v) {
    const auto it=frontier_manager_ptr_->observations_.find(v.get());
    return it!=frontier_manager_ptr_->observations_.end() && it->second->exterior_pass_;
  };
  if (exterior_pending && std::none_of(viewpoints.begin(),viewpoints.end(),exterior)) {
    // Bridge a sparse scan only along the already certified outside pass.
    auto route=published_first_route_;
    if (published_wall_observation_ && published_wall_observation_->exterior_pass_ && route.size()>1 &&
        (route.back()-current_body_position).norm()>planner_manager_->topo_graph_->bubble_min_radius_ &&
        reanchorContour(route,current_body_position,true) && frontier_manager_ptr_->certifyFlightGuide(route)) {
      auto v=std::make_shared<TopoNode>();v->center_=route.back();v->is_viewpoint_=true;
      const auto edge=(route.back()-route[route.size()-2]).eval();v->yaw_=std::atan2(edge.y(),edge.x());
      auto o=std::make_shared<ClusterInfo>(*published_wall_observation_);o->contour_approach_=std::move(route);
      frontier_manager_ptr_->observations_[v.get()]=o;viewpoints.push_back(v);
    } else {
      frontier_manager_ptr_->publishSurveyViewpoints(viewpoints);
      ed_->global_tour_.clear();
      ROS_WARN_THROTTLE(2., "Exterior circuit pending: no measured continuation at %.2f %.2f",current_body_position.x(),current_body_position.y());
      return FRONTIER_PENDING;
    }
  }
  if (!exterior_pending) {
    auto &front=*frontier_manager_ptr_;
    appendDeferredObservation(current_body_position,viewpoints);
    viewpoints.erase(std::remove_if(viewpoints.begin(),viewpoints.end(),[&](const auto &v) {
      const auto it=front.observations_.find(v.get());
      if (it==front.observations_.end() || !front.spentSurveyObservation(*it->second,v->center_)) return false;
      if (std::getenv("EPIC_TRACE_WALL_CHOICE"))
        ROS_INFO("Skipped already surveyed passage observation at %.2f %.2f",v->center_.x(),v->center_.y());
      return true;
    }),viewpoints.end());
  }
  frontier_manager_ptr_->publishSurveyViewpoints(viewpoints);
  // A missing sweep sample can be bridged by the last measured route.
  // Once a new observation exists, let the first-edge policy choose it.
  if (viewpoints.empty() && published_first_leg_ && frontier_manager_ptr_->surface_continuity_) {
    const auto &goal=published_first_leg_->second;
    const Vector3f remaining=goal-current_body_position;
    const auto incoming=frontier_manager_ptr_->recentFlightDirection(current_body_position);
    auto committed=published_first_route_;
    if (incoming && remaining.head<2>().dot(*incoming)>planner_manager_->topo_graph_->bubble_min_radius_ &&
        !frontier_manager_ptr_->visitedStance(goal) && !frontier_manager_ptr_->surveyedEnclosureStance(goal) &&
        reanchorContour(committed,current_body_position,true)) {
      active_contour_approach_=std::move(committed);
      active_contour_preview_=published_preview_;
      active_sweep_entry_=published_sweep_entry_;
      ed_->global_tour_={current_body_position,goal};
      selection_reason_="Keep the previous observation";
      ed_->global_tour_yaws_={planner_manager_->local_data_.curr_yaw_,std::atan2(remaining.y(),remaining.x())};
      updateGoalNode();
      frontier_empty_since_=-1.;
      return SUCCEED;
    }
  }
  if (!exterior_pending)
    frontier_manager_ptr_->connectForwardAperture(current_body_position,viewpoints);
  const bool recovering=(frontier_manager_ptr_->surface_continuity_ && frontier_manager_ptr_->prolongedRetrace());
  if (recovering) {
    const float safe=planner_manager_->topo_graph_->bubble_min_radius_;
    bool fresh_prefix=false;
    for (const auto &vp:viewpoints) {
      const auto obs=frontier_manager_ptr_->observations_.find(vp.get());
      if (obs==frontier_manager_ptr_->observations_.end()) continue;
      if (obs->second->exterior_pass_) {fresh_prefix=true;break;}
      // The globally sampled native frontier can require crossing previously
      // flown space. Let the normal reachability checks test that candidate
      // before committing to a return just because the local outline is spent.
      if (!obs->second->close_surface_sample_ &&
          !frontier_manager_ptr_->nearFlightPath(vp->center_,1.6f*safe,4.f*safe)) {
        fresh_prefix=true;
        break;
      }
      if (obs->second->close_surface_sample_ &&
          (vp->center_-current_body_position).norm()<9.f*safe &&
          std::abs(vp->center_.z()-current_body_position.z())<safe &&
          !frontier_manager_ptr_->nearFlightPath(vp->center_,1.6f*safe,4.f*safe)) {
        fresh_prefix=true;
        break;
      }
      const auto &path=obs->second->contour_approach_;
      float fresh=0.f;
      for (size_t i=1;i<path.size();++i)
        if (!frontier_manager_ptr_->nearFlightPath(path[i],1.6f*safe,4.f*safe))
          fresh+=(path[i]-path[i-1]).norm();
      if (fresh>4.f*safe) {fresh_prefix=true;break;}
    }
    if (!fresh_prefix) {
      const int status=recoverOrWait(current_body_position,vel.cast<float>());
      if (status==SUCCEED || status==NO_FRONTIER) return status;
    }
  }
  const float clearance=planner_manager_->topo_graph_->bubble_min_radius_+.02f;
  viewpoints.erase(std::remove_if(viewpoints.begin(),viewpoints.end(),[&](const auto &node) {
      if (!node->center_.allFinite() || !planner_manager_->lidar_map_interface_->IsInBox(node->center_)) return true;
      if (!frontier_manager_ptr_->surface_continuity_) return false;
      const auto obs=frontier_manager_ptr_->observations_.find(node.get());
      if (frontier_manager_ptr_->revisitsLowerPass(current_body_position,{node->center_}) ||
          (obs!=frontier_manager_ptr_->observations_.end() &&
           frontier_manager_ptr_->revisitsLowerPass(current_body_position,obs->second->contour_approach_))) return true;
      const bool measured=obs!=frontier_manager_ptr_->observations_.end() &&
          !obs->second->contour_approach_.empty() && (obs->second->aperture_approach_ || obs->second->follows_local_outline_ ||
          (!obs->second->contour_support_.empty() && !frontier_manager_ptr_->unseenSupport(*obs->second).empty()));
      const float stance_clearance=measured ? frontier_manager_ptr_->measuredStanceClearance() : clearance;
      if (planner_manager_->lidar_map_interface_->getDisToOcc(node->center_)<stance_clearance) return true;
      // An en-route stance may observe new surfaces. A remote stance
      // already enclosed by actual lidar sightlines need not trigger a
      // second traversal of the volume to get back to it.
      if (!measured && frontier_manager_ptr_->surveyedFromAround(current_body_position,node->center_)) return true;
      return recovering && !(measured && obs->second->exterior_pass_) && frontier_manager_ptr_->nearFlightPath(node->center_,1.25f*clearance,4.f*clearance);
  }),viewpoints.end());
  if (viewpoints.empty()) {
    if (exterior_pending) return FRONTIER_PENDING;
    ed_->global_tour_.clear();
    return recoverOrWait(current_body_position,vel.cast<float>());
  }

  epic_ros::Time t1 = epic_ros::Time::now();
  planner_manager_->topo_graph_->insertNodes(viewpoints, false);
  for (const auto &vp:viewpoints) {
    auto it=frontier_manager_ptr_->observations_.find(vp.get());
    if (it==frontier_manager_ptr_->observations_.end() || it->second->contour_approach_.empty()) continue;
    auto graph=planner_manager_->topo_graph_;
    auto body=graph->odom_node_;
    auto route=it->second->contour_approach_;
    route.insert(route.begin(),body->center_);
    auto checked=route;
    if (!guideSupported(route)) continue;
    if (it->second->aperture_approach_) {
      if (!frontier_manager_ptr_->certifyFlightGuide(route)) continue;
    } else if (!graph->parallel_bubble_astar_->collisionCheck_shortenPath(checked)) continue;
    double length;graph->parallel_bubble_astar_->calculatePathCost(route,length);
    body->neighbors_.insert(vp);vp->neighbors_.insert(body);
    body->paths_[vp]=route;vp->paths_[body]=vector<Eigen::Vector3f>(route.rbegin(),route.rend());
    body->weight_[vp]=vp->weight_[body]=length;
  }
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
  bool last_goal_reachable = dis2last_goal < 2e3;
  // last_goal_reachable = false;

  if (last_goal_reachable && (dis2last_goal < 1.5 * last_frame_value)) {
    last_frame_value = dis2last_goal;
  } else {
    last_goal_reachable = false;
  }

  epic_ros::Time t_start_cvp_1 = epic_ros::Time::now();
  omp_set_num_threads(epic_ros::planningWorkerCount(4));
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 0; i < viewpoints.size(); ++i) {
    if (last_goal_reachable) {
      distance_lastgoal2vp[i] =
          getPathCost(ed_->next_goal_node_, Eigen::Vector3d::Zero(),
                      viewpoints[i]->yaw_, viewpoints[i], viewpoints[i]->yaw_);
      distance_odom2vp[i] =
          getPathCost(planner_manager_->topo_graph_->odom_node_, vel, curr_yaw,
                      viewpoints[i], viewpoints[i]->yaw_,
                      use_velocity_direction_cost_ ? &initial_turn_cost[i] : nullptr);

    } else {
      distance_lastgoal2vp[i] =
          getPathCost(planner_manager_->topo_graph_->odom_node_, vel, curr_yaw,
                      viewpoints[i], viewpoints[i]->yaw_,
                      use_velocity_direction_cost_ ? &initial_turn_cost[i] : nullptr);
      distance_odom2vp[i] = distance_lastgoal2vp[i];
    }
  }
  // These approaches already start at the current measured body. A cached
  // topology path to the same observation may be stale or take a different turn.
  for (size_t i=0;i<viewpoints.size();++i) {
    const auto it=frontier_manager_ptr_->observations_.find(viewpoints[i].get());
    if (it==frontier_manager_ptr_->observations_.end()) continue;
    const auto &guide=it->second->contour_approach_;
    if (guide.size()<2 || (guide.front()-current_body_position).norm()>.1f || !guideSupported(guide)) continue;
    double length;
    planner_manager_->parallel_path_finder_->calculatePathCost(guide,length);
    distance_odom2vp[i]=length/std::max(1.,ep_->v_max_/2.);
    distance_lastgoal2vp[i]=std::min(distance_lastgoal2vp[i],dis2last_goal+distance_odom2vp[i]);
    if (use_velocity_direction_cost_) initial_turn_cost[i]=initialDirectionCost(guide,vel,ep_->w_vdir_,
            planner_manager_->gcopter_config_->yaw_max_vel);
  }
  epic_ros::Time t_end_cvp_1 = epic_ros::Time::now();
  if (bm_without_topo) {
    omp_set_num_threads(epic_ros::planningWorkerCount(4));
    // clang-format off
    #pragma omp parallel for
    // clang-format on
    for (int i = 0; i < viewpoints.size(); ++i) {
      if (last_goal_reachable) {
        distance_lastgoal2vp[i] = getPathCostWithoutTopo(
            ed_->next_goal_node_, Eigen::Vector3d::Zero(), viewpoints[i]->yaw_,
            viewpoints[i], viewpoints[i]->yaw_);
        distance_odom2vp[i] = getPathCostWithoutTopo(
            planner_manager_->topo_graph_->odom_node_, vel, curr_yaw,
            viewpoints[i], viewpoints[i]->yaw_);

      } else {
        distance_lastgoal2vp[i] = getPathCostWithoutTopo(
            planner_manager_->topo_graph_->odom_node_, vel, curr_yaw,
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

  if (std::getenv("EPIC_TRACE_WALL_CHOICE")) {
    ROS_INFO("Viewpoints: %zu sampled, %zu submitted, %zu reachable",
        sampled_views,viewpoints.size(),viewpoint_reachable.size());
    for (const auto &v:viewpoints) {
      const auto it=frontier_manager_ptr_->observations_.find(v.get());
      if (it==frontier_manager_ptr_->observations_.end()) continue;
      const auto &o=*it->second;
      ROS_INFO("Candidate %.2f %.2f %.2f outline %d reachable %d preview %zu",
          v->center_.x(),v->center_.y(),v->center_.z(),int(o.follows_local_outline_),
          int(std::find(viewpoint_reachable.begin(),viewpoint_reachable.end(),v)!=viewpoint_reachable.end()),
          o.contour_preview_.size());
    }
  }

  // A sampled viewpoint can belong to a disconnected topology component even
  // though free space connects it to the robot. Rotate the first repair
  // candidate so one inaccessible viewpoint cannot exhaust every update.
  if (exterior_pending && std::none_of(viewpoint_reachable.begin(),viewpoint_reachable.end(),exterior)) {
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    ed_->global_tour_.clear();
    return FRONTIER_PENDING;
  }
  if (viewpoint_reachable.empty() && !viewpoints.empty()) {
    auto graph=planner_manager_->topo_graph_;
    std::stable_sort(viewpoints.begin(),viewpoints.end(),[&](const auto &a,const auto &b){
      return (a->center_-current_body_position).squaredNorm()<(b->center_-current_body_position).squaredNorm();});
    vector<Eigen::Vector3f> connector;
    auto astar=graph->parallel_bubble_astar_;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(120);
    for (size_t tried=0;tried<viewpoints.size();++tried) {
      if (std::chrono::steady_clock::now()>=deadline) break;
      const auto &target=viewpoints[next_repair_candidate_%viewpoints.size()];
      next_repair_candidate_=(next_repair_candidate_+1)%viewpoints.size();
      const auto &departure=graph->odom_node_->center_;
      bool connected=astar->search(departure,target->center_,connector,.025,false)==ParallelBubbleAstar::REACH_END &&
                     connector.size()>=2 && astar->collisionCheck_shortenPath(connector) && guideSupported(connector);
      if (!connected && frontier_manager_ptr_->surface_continuity_) {
        connected=(frontier_manager_ptr_->connectFlightHistory(departure,target->center_,.025,connector) ||
                   frontier_manager_ptr_->connectObserved(departure,target->center_,.015,connector)) &&
                  guideSupported(connector);
      }
      // A free connector can descend through a finished survey even when
      // the endpoint is outside the visitation radius. Price the same
      // executable edge here that SUPER will receive after selection.
      if (connected && frontier_manager_ptr_->revisitsLowerPass(current_body_position,connector))
        continue;
      if (connected) {
        double length;astar->calculatePathCost(connector,length);
        graph->odom_node_->neighbors_.insert(target);target->neighbors_.insert(graph->odom_node_);
        graph->odom_node_->paths_[target]=connector;
        target->paths_[graph->odom_node_]=vector<Eigen::Vector3f>(connector.rbegin(),connector.rend());
        graph->odom_node_->weight_[target]=target->weight_[graph->odom_node_]=length;
        viewpoint_reachable.push_back(target);
        viewpoint_reachable_distance.push_back(length/(ep_->v_max_/2.));
        viewpoint_reachable_distance2.push_back(viewpoint_reachable_distance.back());
        viewpoint_reachable_turn_cost.push_back(0.);
        ROS_INFO("Reconnected native frontier viewpoint through collision-checked free space");
        break;
      }
    }
  }
  if (viewpoint_reachable.empty()) {
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    ed_->global_tour_.clear();
    return recoverOrWait(current_body_position,vel.cast<float>());
  }

  frontier_empty_since_=-1.;
  next_repair_candidate_=0;
  opening_guide_.clear();
  const auto commitmentFor=[&](const TopoNode::Ptr &view,const ClusterInfo &observation)->std::optional<Vector3f> {
    if (!observation.contour_entry_) return {};
    const auto &primary=frontier_manager_ptr_->perimeter_target_;
    const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
    const bool attached=primary && (view->center_-*primary).norm()<.05f;
    // A remote alternative is a tour destination. Freezing the entire
    // transit to its entry skips newly sampled openings along the way.
    if (attached || (*observation.contour_entry_-current_body_position).norm()<=3.f*radius)
      return observation.contour_entry_;
    return {};
  };
  // Extend the selected first edge only when its nearly exhausted stance
  // lies directly on a longer measured approach. Competing observations
  // still participate in the tour, and a selected corner keeps its preview.
  const auto extendShortObservation=[&](const TopoNode::Ptr &node) {
    if (recovering || !frontier_manager_ptr_->surface_continuity_) return;
    const auto &obs=*frontier_manager_ptr_->observations_.at(node.get());
    const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
    const Eigen::Vector2f motion=vel.head<2>().cast<float>();
    const Eigen::Vector2f offset=(node->center_-current_body_position).head<2>();
    const float speed=motion.norm();
    const float lead=2.f*radius+.8f*speed+.5f*speed*speed/std::max(1.,ep_->a_avg_);
    if (right_hand_priority_ && !obs.exterior_pass_ && offset.norm()<lead &&
        (obs.follows_local_outline_ || obs.close_surface_sample_ || !obs.contour_support_.empty())) {
      auto approach=active_contour_approach_;
      const auto &native=planner_manager_->topo_graph_->odom_node_->paths_;
      const auto path=native.find(node);
      if (approach.size()<2 && path!=native.end()) approach=path->second;
      if (approach.size()>1 && (approach.back()-node->center_).norm()<.01f) {
        const Vector2f tangent=(approach.back()-approach[approach.size()-2]).head<2>();
        float existing_length=0.f;
        for (size_t i=1;i<active_contour_preview_.size();++i)
          existing_length+=(active_contour_preview_[i]-active_contour_preview_[i-1]).norm();
        if (existing_length<2.f*radius) {
          auto preview=frontier_manager_ptr_->previewRightWall(node->center_,tangent);
          if (preview.size()>1 && guideSupported(preview) &&
              !frontier_manager_ptr_->revisitsLowerPass(current_body_position,preview)) {
            active_contour_preview_=std::move(preview);
            ROS_INFO("Sampled a measured right-wall continuation beyond the selected observation");
            return;
          }
        }
      }
    }
    if ((!obs.close_surface_sample_ && obs.contour_support_.empty()) || obs.contour_entry_ ||
        speed<=1.f || offset.norm()>=lead ||
        offset.dot(motion)<=.85f*offset.norm()*speed ||
        hasBoundaryTurn(obs,motion,radius,*frontier_manager_ptr_)) return;
    vector<Eigen::Vector3f> ahead;
    if (!frontier_manager_ptr_->observedBoundaryGuide(current_body_position,ahead,&offset,nullptr,offset.norm()+4.f*radius)) return;
    const Eigen::Vector3f span=ahead.back()-current_body_position;
    const Eigen::Vector2f axis=offset.normalized();
    const float along=span.head<2>().dot(axis);
    if (along<=offset.norm()+4.f*radius || (span.head<2>()-along*axis).norm()>=radius+.3f*along ||
        std::abs(span.z())>=1.5f*radius || distanceToPolyline(node->center_,ahead)>=radius) return;
    for (const auto &point:obs.contour_approach_)
      if (distanceToPolyline(point,ahead)>radius) return;
    for (const auto &point:active_contour_preview_)
      if (distanceToPolyline(point,ahead)>radius) return;
    // SUPER can fly through the first observation, while the next ATSP
    // update is free to select a new corner anywhere along the preview.
    size_t nearest=1;
    float distance=std::numeric_limits<float>::infinity();
    for (size_t i=1;i<ahead.size();++i) {
      const Eigen::Vector3f edge=ahead[i]-ahead[i-1];
      const float fraction=std::clamp((node->center_-ahead[i-1]).dot(edge)/std::max(.0001f,edge.squaredNorm()),0.f,1.f);
      const float error=(ahead[i-1]+fraction*edge-node->center_).squaredNorm();
      if (error<distance) {distance=error;nearest=i;}
    }
    vector<Eigen::Vector3f> preview{node->center_};
    for (size_t i=nearest;i<ahead.size();++i)
      if ((preview.back()-ahead[i]).norm()>.01f) preview.push_back(ahead[i]);
    if (preview.size()>1 && frontier_manager_ptr_->certifyFlightGuide(preview)) {
      active_contour_preview_.swap(preview);
      ROS_INFO("Extended SUPER preview beyond selected viewpoint over %.1fm of measured air",along);
    }
  };
  // Revalidate the issued first leg before replacing it with a shorter
  // observation on that same line or briefly reversing at a new entrance.
  const auto keeps_issued_pass=[&](const TopoNode::Ptr &next) {
    auto &front=*frontier_manager_ptr_;
    const auto &chosen=*front.observations_.at(next.get());
    if (chosen.exterior_pass_ || !front.surface_continuity_ || !published_first_leg_ ||
        published_first_route_.size()<2) return false;
    const float radius=planner_manager_->topo_graph_->bubble_min_radius_;
    const auto &[start,goal]=*published_first_leg_;
    const Vector3f remaining=goal-current_body_position,proposal=next->center_-current_body_position;
    if (remaining.head<2>().norm()<2.f*radius || std::abs(remaining.z())>radius ||
        remaining.dot(goal-start)<=0.f || (next->center_-goal).norm()<radius) return false;
    const auto native=planner_manager_->topo_graph_->odom_node_->paths_.find(next);
    auto guide=chosen.contour_approach_.empty() && native!=planner_manager_->topo_graph_->odom_node_->paths_.end()
        ? native->second : chosen.contour_approach_;
    if (guide.size()<2) return false;
    guide.front()=current_body_position;
    const Vector2f motion=vel.head<2>().cast<float>();
    const float progress=proposal.head<2>().dot(remaining.head<2>().normalized());
    const bool on_pass=motion.norm()>1.f && motion.dot(remaining.head<2>())>.8f*motion.norm()*remaining.head<2>().norm() &&
        (progress>=-radius || front.nearFlightPath(next->center_,1.5f*radius,0.f,.35f*radius)) &&
        !hasBoundaryTurn(chosen,motion,radius,front) &&
        retractsStraightGoal(published_first_route_,guide,chosen.contour_preview_,radius);
    if (!on_pass) {
      const auto incoming=front.recentFlightDirection(current_body_position);
      if (recovering || epic_ros::Time::now().toSec()>=departure_deadline_ || front.visitedStance(goal) ||
          progress>=-.15f*proposal.head<2>().norm() || !incoming ||
          (front.flownObservationStance(goal) && !front.flownObservationStance(next->center_)) ||
          front.surveyDeparture(current_body_position,*incoming,guide,false,chosen.right_hand_opening_)!=
              FrontierManager::Departure::Backwards || distanceToPolyline(goal,guide)<radius) return false;
    }
    auto committed=published_first_route_;
    if (!reanchorContour(committed,current_body_position,true) || !front.certifyFlightGuide(committed)) return false;
    active_contour_approach_=std::move(committed);
    active_contour_preview_=published_preview_;
    active_sweep_entry_=published_sweep_entry_;
    if (on_pass) {
      // SUPER will cross this near stance on the retained leg. Leaving it
      // as #2 would immediately preview a trip back after the retained #1.
      ed_->global_tour_.erase(ed_->global_tour_.begin()+1);
      ed_->global_tour_yaws_.erase(ed_->global_tour_yaws_.begin()+1);
    }
    const auto existing=std::find_if(ed_->global_tour_.begin()+1,ed_->global_tour_.end(),[&](const Vector3f &p) {
      return (p-goal).norm()<.01f;
    });
    if (existing!=ed_->global_tour_.end()) {
      ed_->global_tour_yaws_.erase(ed_->global_tour_yaws_.begin()+(existing-ed_->global_tour_.begin()));
      ed_->global_tour_.erase(existing);
    }
    ed_->global_tour_.insert(ed_->global_tour_.begin()+1,goal);
    ed_->global_tour_yaws_.insert(ed_->global_tour_yaws_.begin()+1,std::atan2(remaining.y(),remaining.x()));
    planner_manager_->local_data_.end_yaw_=ed_->global_tour_yaws_[1];
    updateGoalNode();
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    selection_reason_=on_pass ? "Continue the selected pass" : "Keep the entry instead of reversing";
    ROS_INFO("Retained certified first observation at %.2f %.2f instead of %.2f %.2f (%s)",
        goal.x(),goal.y(),next->center_.x(),next->center_.y(),on_pass?"same pass":"reverse at entrance");
    return true;
  };
  if (viewpoint_reachable.size() == 1) {
    ed_->global_tour_.clear();
    ed_->global_tour_yaws_ = {curr_yaw, viewpoint_reachable.front()->yaw_};
    ed_->global_tour_.emplace_back(current_body_position);
    ed_->global_tour_.emplace_back(viewpoint_reachable.front()->center_);
    if (keeps_issued_pass(viewpoint_reachable.front())) return SUCCEED;
    selection_reason_=restored_observation_ && (viewpoint_reachable.front()->center_-*restored_observation_).norm()<.01f ?
        "Only reachable view | restored from memory" : "Only reachable view";
    if (frontier_manager_ptr_->surface_continuity_) {
      selected_observation_=frontier_manager_ptr_->observations_.at(viewpoint_reachable.front().get());
      const auto &chosen=*selected_observation_;
      active_sweep_entry_=commitmentFor(viewpoint_reachable.front(),chosen);
      active_contour_approach_=chosen.contour_approach_;
      active_contour_preview_=chosen.contour_preview_;
      extendShortObservation(viewpoint_reachable.front());
    }
    planner_manager_->local_data_.end_yaw_ = viewpoint_reachable.front()->yaw_;
    updateGoalNode(viewpoint_reachable.front());
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    return SUCCEED;
  }

  int dim = viewpoint_reachable.size() + 1;
  vector<const char *> first_edge_reason(dim,"ATSP");
  vector<vector<Vector3f>> forward_connectors(dim);
  Eigen::MatrixXd mat;
  mat.resize(dim, dim);
  mat.setZero();
  for (int i = 1; i < dim; ++i) {
    mat(0, i) = frontier_manager_ptr_->surface_continuity_
        ? viewpoint_reachable_distance2[i-1] : viewpoint_reachable_distance[i-1];
  }

  omp_set_num_threads(epic_ros::planningWorkerCount(4));
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 1; i < dim; i++) {
    for (int j = i + 1; j < dim; j++) {
      double cost=getPathCost(viewpoint_reachable[i-1],Eigen::Vector3d::Zero(),
          viewpoint_reachable[i-1]->yaw_,viewpoint_reachable[j-1],viewpoint_reachable[j-1]->yaw_,
          nullptr);
      if (frontier_manager_ptr_->surface_continuity_) {
        // Both views have validated routes from the body. A missing direct
        // graph edge must not dominate the ATSP ordering with a 2000s penalty.
        const double via_body=viewpoint_reachable_distance2[i-1]+viewpoint_reachable_distance2[j-1];
        cost=std::min(cost,via_body);
      }
      mat(i,j)=mat(j,i)=cost;
    }
  }
  for (int i=1;i<dim;++i) mat(i,0)=2e3-viewpoint_reachable_distance2[i-1]*.2;
  // In rolling flight the first edge must be the executable body-to-view
  // route. A shortcut through another candidate misprices that first choice.
  for (int i = frontier_manager_ptr_->surface_continuity_ ? 1 : 0; i < dim; ++i) {
    for (int j = 1; j < dim; ++j) {
      for (int k = 1; k < dim; ++k) {
        if (mat(i, j) > mat(i, k) + mat(k, j)) {
          mat(i, j) = mat(i, k) + mat(k, j) + 1e-2;
        }
      }
    }
  }
  const auto &observations = frontier_manager_ptr_->observations_;
  if (use_velocity_direction_cost_)
    for (int i=1;i<dim;++i) mat(0,i)+=viewpoint_reachable_turn_cost[i-1];
  if (frontier_manager_ptr_->surface_continuity_) {
    // Only the initial part of a receding tour is executed. Score its marginal
    // support per flight second here; adding a constant reward to every node
    // would leave the order of a complete ATSP tour unchanged.
    using Cells = std::unordered_set<Eigen::Vector3i,Vector3i_Hash>;
    std::vector<Cells> support(dim);
    for (int i=1;i<dim;++i) {
      const auto &obs=*observations.at(viewpoint_reachable[i-1].get());
      auto cells=frontier_manager_ptr_->unseenSupport(obs);
      support[i].insert(cells.begin(),cells.end());
    }
    const double cruise=std::max(1.,ep_->v_max_/2.);
    const double horizon=frontier_manager_ptr_->sensorUpdateRange()/cruise;
    std::vector<double> merit(dim);
    for (int i=1;i<dim;++i) {
      const auto &first=*viewpoint_reachable[i-1];
      const auto &obs=*observations.at(viewpoint_reachable[i-1].get());
      const double first_time=std::max(.25,viewpoint_reachable_distance2[i-1]+viewpoint_reachable_turn_cost[i-1]);
      const double first_gain=std::log1p(support[i].size());
      double rate=first_gain/(1.+first_time);
      for (int j=1;j<dim;++j) {
        if (i==j || mat(i,j)>=2e3) continue;
        size_t extra=0;
        for (const auto &cell:support[j]) if (!support[i].count(cell)) ++extra;
        const Eigen::Vector3f a=obs.contour_approach_.size()>1 ?
            (obs.contour_approach_.back()-obs.contour_approach_[obs.contour_approach_.size()-2]).eval() :
            (first.center_-current_body_position).eval();
        const Eigen::Vector3f b=viewpoint_reachable[j-1]->center_-first.center_;
        const double bend=(a.norm()>.1f && b.norm()>.1f) ?
            std::acos(std::clamp(double(a.normalized().dot(b.normalized())),-1.,1.)) : 0.;
        const double next_time=mat(i,j)+bend/std::max(.1,ep_->yaw_v_max_);
        rate=std::max(rate,(first_gain+.6*std::log1p(extra))/(1.+first_time+next_time));
      }
      // A reachable right-hand continuation and the already published goal
      // earn bounded time bonuses, so a productive alternative can overtake.
      const bool continuing=published_first_leg_ &&
          (published_first_leg_->second-first.center_).norm()<frontier_manager_ptr_->getCellSize();
      const double inspected_time=obs.fresh_contour_length_/cruise;
      // Postpone vertical excursions while a near-level observation remains
      // productive. A reachable ascent still competes once this band is spent.
      const double vertical_time=std::abs(first.center_.z()-current_body_position.z())/cruise;
      mat(0,i)+=std::min(horizon,2.*vertical_time);
      merit[i]=horizon*std::min(1.,rate)+std::min(horizon,2.*inspected_time)+
          (continuing ? .2*horizon:0.);
    }
    const float survey_radius=planner_manager_->topo_graph_->bubble_min_radius_;
    const double span=mat.block(1,1,dim-1,dim-1).maxCoeff();
    const double end_spread=mat.col(0).tail(dim-1).maxCoeff()-mat.col(0).tail(dim-1).minCoeff();
    const double first_edge_bound=dim*(1.+std::max(span,mat.row(0).tail(dim-1).maxCoeff())+horizon)+end_spread;
    // The first edge follows one ordering. Later edges remain the ordinary
    // open tour, and a reverse transfer is available when the local pass ends.
    auto &front=*frontier_manager_ptr_;
    const auto incoming=front.recentFlightDirection(current_body_position);
    const Vector2f body_forward(std::cos(curr_yaw),std::sin(curr_yaw));
    const Vector2f body_right(body_forward.y(),-body_forward.x());
    const Vector2f heading=incoming.value_or(body_forward);
    const auto band=front.currentSurveyBand(current_body_position);
    const auto &odom=planner_manager_->topo_graph_->odom_node_;
    struct FirstEdge {
      bool exterior,same_level,unflown,right_opening,continues_pass,attached,right_wall,wall;
      FrontierManager::Departure departure;
      int direction_priority=0;
      bool rolling_wall=false;
      auto priority() const {
        return std::make_tuple(exterior,rolling_wall,attached && right_wall,direction_priority,same_level,unflown,right_opening,
                               continues_pass,attached,right_wall,wall,departure);
      }
    };
    std::vector<FirstEdge> ranks(dim);
    auto course=front.surveyFlightDirection(current_body_position);
    const Vector2f velocity=vel.head<2>().cast<float>();
    if (velocity.norm()>1.f && (!course || velocity.dot(*course)>0.f))
      course=velocity.normalized();
    const float braking_distance=vel.head<2>().squaredNorm()/(2.f*std::max(.1,ep_->a_avg_));
    const float rolling_span=std::clamp(braking_distance+2.f*survey_radius,3.f*survey_radius,
                                        std::max(3.f*survey_radius,.5f*front.sensorUpdateRange()));
    vector<Vector3f> forward_goals;
    vector<int> right_side_samples,returning_through_entry;
    const auto crossing=entered_turn_ ? entered_turn_ : front.recentRightTurn(current_body_position);
    bool has_interior_wall=false;
    bool has_left_branch=false;
    const Vector2f right(heading.y(),-heading.x());
    for (int i=1;i<dim;++i) {
      const auto &node=viewpoint_reachable[i-1];
      const auto &obs=*observations.at(node.get());
      const vector<Vector3f> *guide=&obs.contour_approach_;
      const auto native=odom->paths_.find(node);
      if (guide->empty() && native!=odom->paths_.end()) guide=&native->second;
      const bool wall=!support[i].empty() && (!obs.visible_cells_.empty() ||
          std::any_of(obs.contour_support_.begin(),obs.contour_support_.end(),[](const auto &s) {
            return s.kind==ObservationSample::Kind::WallFace;
          }));
      auto departure_path=*guide;
      if (!departure_path.empty() && obs.contour_preview_.size()>1 &&
          (departure_path.back()-obs.contour_preview_.front()).norm()<survey_radius)
        departure_path.insert(departure_path.end(),obs.contour_preview_.begin()+1,obs.contour_preview_.end());
      const bool right_wall=front.followsRightWall(departure_path);
      const bool fresh=(right_wall && !front.visitedStance(node->center_)) || !support[i].empty() ||
          obs.fresh_contour_length_>=survey_radius;
      const bool level=fresh && (!band || (std::abs(node->center_.z()-*band)<=survey_radius &&
          std::all_of(guide->begin(),guide->end(),[&](const auto &p){
            return front.withinLevelApproach(p.z(),current_body_position.z(),*band,survey_radius);
          })));
      auto departure=front.surveyDeparture(current_body_position,heading,*guide,wall,
          fresh && (obs.right_hand_opening_ || (wall && obs.contour_entry_)));
      // A temporary native graph connection can disappear between scans.
      // An observation on the certified wall preview still belongs to that pass.
      if (fresh && departure==FrontierManager::Departure::Transfer && published_preview_.size()>1 &&
          distanceToPolyline(node->center_,published_preview_)<survey_radius &&
          (node->center_-current_body_position).head<2>().dot(heading)>survey_radius &&
          !front.visitedStance(node->center_) && front.certifyFlightGuide(published_preview_))
        departure=FrontierManager::Departure::AlongWall;
      const bool moving_away=incoming && departure==FrontierManager::Departure::Backwards;
      // While acquiring a boundary, prefer an actual observation to a
      // sideways crossing supported only by holes in the free-space map.
      // The native outline itself verifies an upright right face at two
      // elevations. Sparse centre-height returns at a bend need not repeat it.
      const bool attached=obs.follows_local_outline_ && fresh && !moving_away;
      // A right turn into a previously surveyed passage is a transfer. Pick a
      // reachable new stance first, even if its approach turns left at the fork.
      const Vector2f offset=(node->center_-current_body_position).head<2>();
      // The end of an unfinished wall can share a doorway with an older
      // passage. The outline sampler measures unflown length along the wall.
      // Keep those metres in the tour even when the endpoint was flown near.
      const bool unfinished_wall=attached && incoming && right_wall &&
          obs.fresh_contour_length_>=2.f*survey_radius && offset.dot(*incoming)>2.f*survey_radius &&
          departure>=FrontierManager::Departure::Forward &&
          front.certifyFlightGuide(*guide);
      const bool new_pass=fresh && (!front.flownObservationStance(node->center_) || unfinished_wall);
      // At a real fork a new, reachable right-hand entrance wins even if
      // the opposite arm currently has a longer continuous wall return.
      const bool right_opening=fresh && departure==FrontierManager::Departure::RightOpening &&
          (wall || right_wall || obs.right_hand_opening_);
      // Two reachable, fresh stances can expose a fork before either gets
      // tagged as an opening. Check their lateral separation relative to
      // entry, and require measured free air into the actual right arm.
      if (incoming && new_pass && level && !front.visitedStance(node->center_) && !support[i].empty()) {
        const Vector2f offset=(node->center_-current_body_position).head<2>();
        const float side=offset.dot(right);
        if (offset.dot(heading)>=-survey_radius && std::abs(side)>1.5f*survey_radius &&
            std::abs(side)>.45f*offset.norm()) {
          if (side<0.f) has_left_branch=true;
          else if (!right_opening) {
            vector<Vector3f> arm=*guide;
            const bool checked=arm.size()>1 && reanchorContour(arm,current_body_position,true);
            if ((checked || front.connectObserved(current_body_position,node->center_,.012,arm)) &&
                front.certifyFlightGuide(arm) &&
                front.surveyDeparture(current_body_position,heading,arm,wall,true)==FrontierManager::Departure::RightOpening)
              right_side_samples.push_back(i);
          }
        }
      }
      bool level_pass=level;
      if (course && new_pass && !right_opening) {
        const Vector3f delta=node->center_-current_body_position;
        const Vector2f extent=delta.head<2>();
        if (std::abs(delta.z())<=survey_radius && extent.norm()>rolling_span+2.f*survey_radius &&
            extent.dot(*course)>.85f*extent.norm()) {
          const Vector2f axis=extent.normalized();
          const auto stays_on_pass=[&](const vector<Vector3f> &checked) {
            return checked.size()>1 && std::all_of(checked.begin(),checked.end(),[&](const Vector3f &p) {
                const Vector3f offset=p-current_body_position;
                const float along=offset.head<2>().dot(axis);
                return along>=-survey_radius && std::abs(offset.z())<=survey_radius &&
                       (offset.head<2>()-along*axis).norm()<=2.f*survey_radius;
              }) && front.certifyFlightGuide(checked);
          };
          vector<Vector3f> checked=*guide;
          bool connected=checked.size()>1 && reanchorContour(checked,current_body_position,true) && stays_on_pass(checked);
          // A valid graph edge can leave the current level and rejoin it.
          // Try the measured local connection before ranking a reverse transfer.
          if (!connected && front.connectObserved(current_body_position,node->center_,.012,checked) && stays_on_pass(checked)) {
            connected=true;
            forward_connectors[i]=checked;
          }
          if (connected) {
            forward_goals.push_back(node->center_);
            level_pass=true;
          }
        }
      }
      if (crossing && level && new_pass && !obs.exterior_pass_ && guide->size()>1) {
        const auto &[entry,inward]=*crossing;
        const float depth=(node->center_-entry).head<2>().dot(inward);
        auto approach=*guide;
        approach.front()=current_body_position;
        const bool exits=exitsThroughEntry(approach,entry,inward,survey_radius);
        if (!exits && wall && depth>survey_radius && !front.flownObservationStance(node->center_) &&
            front.certifyFlightGuide(approach)) has_interior_wall=true;
        if (exits) returning_through_entry.push_back(i);
      }
      ranks[i]={obs.exterior_pass_,level_pass,new_pass,right_opening,true,attached,right_wall && !moving_away,
          wall && !moving_away,fresh ? departure : FrontierManager::Departure::Transfer};
      if (right_hand_priority_ && level_pass && new_pass) {
        const Vector2f target=(node->center_-current_body_position).head<2>();
        const auto first=std::find_if(guide->begin(),guide->end(),[&](const auto &point) {
          return (point-current_body_position).template head<2>().norm()>=.5f*survey_radius;
        });
        if (first!=guide->end()) {
          const Vector2f step=(*first-current_body_position).head<2>();
          const float angle=std::atan2(target.dot(body_right),target.dot(body_forward));
          if (step.dot(body_forward)>=-.25f*step.norm()) {
            if (angle>=.12f && angle<=1.31f) ranks[i].direction_priority=2;
            else if (angle>=-.35f && angle<1.57f) ranks[i].direction_priority=1;
            // Prefer a measured wall path with enough ordered arc to keep rolling.
            // A short waypoint with the same bearing cannot supply that preview.
            if (!moving_away && (attached || right_wall) &&
                (wall || obs.fresh_contour_length_>=survey_radius) && departure_path.size()>1) {
              float arc=0.f;
              bool forward_route=true;
              for (size_t k=1;k<departure_path.size();++k) {
                const Vector3f edge=departure_path[k]-departure_path[k-1];
                arc+=edge.norm();
                if (k>1) {
                  const Vector3f previous=departure_path[k-1]-departure_path[k-2];
                  if (edge.dot(previous)<-.35f*edge.norm()*previous.norm()) {
                    forward_route=false;
                    break;
                  }
                }
              }
              ranks[i].rolling_wall=forward_route && arc>=rolling_span &&
                  front.keepsRightWall(departure_path,
                      std::min(arc,rolling_span+frontier_manager_ptr_->getCellSize())) &&
                  guideSupported(departure_path) && front.certifyFlightGuide(departure_path);
              if (ranks[i].rolling_wall)
                merit[i]+=horizon*.3*std::min(2.f,arc/rolling_span);
            }
            if (ranks[i].direction_priority)
              merit[i]+=.15*horizon*std::max(0.f,1.f-std::abs(angle-.65f)/1.2f);

          }
        }
      }
      if (std::getenv("EPIC_TRACE_WALL_CHOICE")) {
        ROS_INFO("First edge to %.2f %.2f %.2f: level %d right wall %d surface %d departure %d unseen cells %zu attached %d new pass %d direction tier %d",
            node->center_.x(),node->center_.y(),node->center_.z(),int(level),int(right_wall),
            int(wall),int(departure),support[i].size(),int(attached),int(new_pass),ranks[i].direction_priority);
      }
    }
    if (has_left_branch) {
      for (const int i:right_side_samples) {
        ranks[i].right_opening=true;
        ranks[i].departure=FrontierManager::Departure::RightOpening;
        ROS_INFO("Selected measured right-side opening tier for %.2f %.2f",
            viewpoint_reachable[i-1]->center_.x(),viewpoint_reachable[i-1]->center_.y());
      }
    }
    for (int i=1;i<dim && !forward_goals.empty();++i) {
      if (ranks[i].right_opening) continue;
      const auto &node=viewpoint_reachable[i-1];
      const auto &sample=*observations.at(node.get());
      if (std::any_of(forward_goals.begin(),forward_goals.end(),[&](const Vector3f &far) {
            return shortOnForwardPass(current_body_position,node->center_,sample.contour_preview_,far,
                                      survey_radius,rolling_span);
          })) {
        ranks[i].continues_pass=false;
        ROS_INFO("Deferred short first observation on a measured forward pass at %.2f %.2f",node->center_.x(),node->center_.y());
      }
    }
    // Two right-opening stances can represent the very same entrance.
    // A clear line through the near one lets SUPER fly to the farther #1
    // without taking a tour edge back out of the branch just entered.
    for (int i=1;i<dim;++i) {
      if (!ranks[i].right_opening || !ranks[i].same_level || !ranks[i].unflown) continue;
      const Vector3f near=viewpoint_reachable[i-1]->center_,entry=near-current_body_position;
      if (entry.norm()<survey_radius || entry.norm()>2.f*rolling_span) continue;
      for (int j=1;j<dim;++j) {
        if (i==j || !ranks[j].right_opening || !ranks[j].same_level || !ranks[j].unflown) continue;
        const Vector3f far=viewpoint_reachable[j-1]->center_,onward=far-near;
        if (onward.norm()<2.f*survey_radius || onward.dot(entry)<.9f*onward.norm()*entry.norm() ||
            distanceToPolyline(near,{current_body_position,far})>survey_radius) continue;
        if (front.certifyFlightGuide({near,far})) {ranks[i].continues_pass=false;break;}
      }
    }
    if (has_interior_wall && !returning_through_entry.empty()) entered_turn_=crossing;
    if (has_interior_wall) for (const int i:returning_through_entry) {
      ranks[i].right_opening=false;
      if (right_hand_priority_) {
        ranks[i].right_wall=false;
        ranks[i].direction_priority=0;
        ranks[i].rolling_wall=false;
      }
      ranks[i].continues_pass=false;
      ROS_INFO("Deferred return through the entered opening while a reachable interior wall remains");
    }
    const auto best_rank=std::max_element(ranks.begin()+1,ranks.end(),[](const FirstEdge &a,const FirstEdge &b) {
      return a.priority()<b.priority();
    });
    for (int i=1;i<dim;++i) if (ranks[i].priority()==best_rank->priority()) merit[i]+=first_edge_bound;
    for (int i=1;i<dim;++i) {
      const auto &rank=ranks[i];
      first_edge_reason[i]=rank.exterior ? "ATSP | perimeter pass" :
          rank.rolling_wall ? "ATSP | continuous forward-right wall" :
          rank.direction_priority==2 ? "ATSP | forward-right observation" :
          rank.right_opening ? "ATSP | right opening" :
          rank.right_wall ? "ATSP | right wall" :
          rank.wall ? "ATSP | inspect wall" :
          rank.departure==FrontierManager::Departure::Forward ? "ATSP | forward observation" :
          rank.same_level && rank.unflown ? "ATSP | unflown view on this level" : "ATSP | travel and observation cost";
    }
    const double best=*std::max_element(merit.begin()+1,merit.end());
    for (int i=1;i<dim;++i) mat(0,i)+=best-merit[i];
  }
  vector<int> indices;
  indices.reserve(dim);
  epic_ros::Time start_tsp = epic_ros::Time::now();
  cout << "calculate tsp cost matrix cost " << (start_tsp - t2).toSec() * 1000
       << "ms" << endl;
  if (!solveLkh(mat, indices)) {
    ROS_WARN_THROTTLE(2., "LKH did not produce a valid tour; waiting for a new exploration plan");
    ed_->global_tour_.clear();
    ed_->global_tour_yaws_.clear();
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    return FRONTIER_PENDING;
  }
  epic_ros::Time end_tsp = epic_ros::Time::now();
  cout << "lkh solver cost: " << (end_tsp - start_tsp).toSec() * 1000 << "ms"
       << endl;
  // if ((end_tsp - start_tsp).toSec() * 1000 > 100)
  //   exit(0);
  ed_->global_tour_.clear();
  ed_->global_tour_yaws_.clear();
  for (auto &i : indices) {
    ed_->global_tour_yaws_.push_back(i == 0 ? curr_yaw : viewpoint_reachable[i-1]->yaw_);
    if (i == 0) {
      // The handoff compares the current pose with the pose used to select #1.
      ed_->global_tour_.push_back(frontier_manager_ptr_->surface_continuity_ ?
          current_body_position : planner_manager_->topo_graph_->odom_node_->center_);
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
  selection_reason_=restored_observation_ && (selected_viewpoint->center_-*restored_observation_).norm()<.01f ?
      "ATSP | restored from memory" : first_edge_reason[indices[1]];
  if (keeps_issued_pass(selected_viewpoint)) return SUCCEED;
  if (frontier_manager_ptr_->surface_continuity_) {
    selected_observation_=observations.at(selected_viewpoint.get());
    const auto &chosen=*selected_observation_;
    const auto &primary=frontier_manager_ptr_->perimeter_target_;
    ROS_INFO("ATSP first %.2f %.2f %.2f body %.2f %.2f primary %d entry %d fresh %.2f boundary samples %zu",
        selected_viewpoint->center_.x(),selected_viewpoint->center_.y(),selected_viewpoint->center_.z(),
        current_body_position.x(),current_body_position.y(),
        int(primary && (*primary-selected_viewpoint->center_).norm()<.05f),int(chosen.contour_entry_.has_value()),
        chosen.fresh_contour_length_,chosen.contour_support_.size());
    active_sweep_entry_=commitmentFor(selected_viewpoint,chosen);
    auto &checked=forward_connectors[indices[1]];
    active_contour_approach_=checked.empty() ? chosen.contour_approach_ : std::move(checked);
    active_contour_preview_=chosen.contour_preview_;
    extendShortObservation(selected_viewpoint);
  }
  updateGoalNode(selected_viewpoint);
  planner_manager_->topo_graph_->removeNodes(viewpoints);

  planner_manager_->local_data_.end_yaw_ =
      viewpoint_reachable[indices[1] - 1]->yaw_;
  return SUCCEED;
}

bool FastExplorationManager::solveLkh(const Eigen::MatrixXd &cost_mat, vector<int> &indices) {
  const int dimension = cost_mat.rows();
  if (dimension < 3 || cost_mat.cols() != dimension || !cost_mat.allFinite()) return false;
  constexpr int scale = 100;
  // LKH's explicit ATSP matrix uses integer weights and multiplies by PRECISION.
  constexpr int precision = 100;
  constexpr int max_weight = std::numeric_limits<int>::max() / 2 / precision;
  if (cost_mat.cwiseAbs().maxCoeff() * scale > max_weight) return false;

  const std::filesystem::path directory(ep_->tsp_dir_);
  const auto problem = directory / "single.tsp";
  const auto parameters = directory / "single.par";
  const auto result = directory / "single.txt";
  std::error_code error;
  std::filesystem::remove(result, error);
  if (error) return false;

  ofstream par_file(parameters);
  par_file << "PROBLEM_FILE = " << problem.string() << "\n"
           << "GAIN23 = NO\nMOVE_TYPE = 2\n"
           << "PRECISION = " << precision << "\n"
           << "OUTPUT_TOUR_FILE = " << result.string() << "\nRUNS = 10\n";
  par_file.close();
  if (!par_file) return false;

  ofstream prob_file(problem);
  prob_file << "NAME : single\nTYPE : ATSP\nDIMENSION : " << dimension
            << "\nEDGE_WEIGHT_TYPE : EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      prob_file << static_cast<int>(cost_mat(i, j) * scale) << ' ';
    }
    prob_file << '\n';
  }
  prob_file << "EOF";
  prob_file.close();
  if (!prob_file || !runLkhWorker(parameters)) return false;
  ifstream res_file(result);
  return epic_ros::readLkhTour(res_file, dimension, indices);
}

bool FastExplorationManager::reanchorLocalPath(vector<Eigen::Vector3f> &path,
    const Eigen::Vector3f &boundary,const Eigen::Vector3f &velocity) {
  if(path.empty() || !boundary.allFinite() || !velocity.allFinite())return false;
  auto astar=planner_manager_->topo_graph_->parallel_bubble_astar_;
  auto lio=planner_manager_->lidar_map_interface_;
  const auto goal=path.back();
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
    ROS_WARN("Cannot safely reanchor cached local guide; native A* status %d",status);
    path.clear();return false;
  }
  path.swap(rebuilt);return true;
}

// SUPER tracks a rolling guide. A later observation on another level must
// not initiate a climb ahead of the current observation. Preserve continuous
// slopes and horizontal bends, including the certified prefix of a new slope.
void FastExplorationManager::limitPreviewSlope(vector<Eigen::Vector3f> &guide,size_t first_goal,float tolerance) {
  if (first_goal==0 || first_goal+1>=guide.size() || tolerance<=0.f) return;
  size_t approach=first_goal-1;
  while (approach>0 && (guide[first_goal]-guide[approach]).head<2>().norm()<4.f*tolerance) --approach;
  const Eigen::Vector3f incoming=guide[first_goal]-guide[approach];
  const float run=incoming.head<2>().norm();
  if (run<tolerance) return;
  const float slope=incoming.z()/run;
  float error=0.f;
  for (size_t i=first_goal+1;i<guide.size();++i) {
    const Eigen::Vector3f edge=guide[i]-guide[i-1];
    const float next=error+edge.z()-slope*edge.head<2>().norm();
    if (std::abs(next)>tolerance) {
      const float fraction=(std::copysign(tolerance,next)-error)/(next-error);
      if (fraction*edge.norm()>.02f) {
        guide[i]=guide[i-1]+fraction*edge;
        guide.resize(i+1);
      } else guide.resize(i);
      return;
    }
    error=next;
  }
}

bool FastExplorationManager::extendTourPath(vector<Eigen::Vector3f> &path, bool preserve_first_vertex) {
  epic_ros::NodeHandle nh;double limit;
  // Keep enough guide for the next connected frontier on measured surveys.
  nh.param("exploration/tour_lookahead_distance",limit,
           frontier_manager_ptr_->surface_continuity_ ? 2.*frontier_manager_ptr_->sensorUpdateRange() : 0.);
  double corner_speed;nh.param("exploration/corridor_corner_preview_speed",corner_speed,0.);
  if(corner_speed>0. && planner_manager_->local_data_.curr_vel_.norm()>corner_speed)return false;
  if(!std::isfinite(limit) || limit<=0. || path.size()<2 ||
      ed_->global_tour_.size()<3 || (path.back()-ed_->global_tour_[1]).norm()>.1f)return false;
  if (!preserve_first_vertex && (frontier_manager_ptr_->perimeter_target_ ||
      !active_contour_approach_.empty())) return false;
  double used=0.;
  for(size_t i=1;i<path.size();++i) {
    if(!path[i].allFinite() || !path[i-1].allFinite())return false;
    used+=(path[i]-path[i-1]).norm();
  }
  if(used>=limit-.1)return false;
  const double first_leg = used;
  auto extended=path;bool changed=false;
  for(size_t j=2;j<ed_->global_tour_.size() && used<limit-.1;++j) {
    auto &front=*frontier_manager_ptr_;
    auto &astar=*planner_manager_->topo_graph_->parallel_bubble_astar_;
    vector<Vector3f> segment;
    const auto advances=[&](const vector<Vector3f> &edge) {
      if (edge.size()<2 || !guideSupported(edge)) return false;
      if (!front.surface_continuity_) return true;
      const Vector3f in=extended.back()-extended[extended.size()-2];
      const auto next=std::find_if(edge.begin()+1,edge.end(),[&](const Vector3f &p) {
        return (p-edge.front()).norm()>.05f;
      });
      if (next==edge.end() || in.norm()<.01f) return false;
      const Vector3f out=*next-edge.front();
      return in.dot(out)>=-.1f*in.norm()*out.norm();
    };
    const auto &destination=ed_->global_tour_[j];
    bool connected=astar.search(extended.back(),destination,segment,.01,false)==ParallelBubbleAstar::REACH_END &&
        segment.size()>1 && astar.collisionCheck_shortenPath(segment) && advances(segment);
    if (!connected && right_hand_priority_ && front.surface_continuity_) {
      // A topology edge can point back through the approach. Recheck the
      // measured free-space connection at the turn before dropping preview.
      connected=front.connectObserved(extended.back(),destination,.015,segment) &&
          advances(segment) && front.certifyFlightGuide(segment) &&
          !front.revisitsLowerPass(extended.front(),segment);
      if (connected) ROS_INFO("Connected the next wall observation from the current turn in measured air");
    }
    if (!connected) break;
    auto prefix=FastPlannerManager::boundedGuidePrefix(segment,limit-used);
    if(prefix.size()<2)break;
    for(size_t i=1;i<prefix.size();++i) {
      used+=(prefix[i]-prefix[i-1]).norm();
      extended.push_back(prefix[i].cast<float>());
    }
    changed=true;
    if(frontier_manager_ptr_->surface_continuity_ || (extended.back()-ed_->global_tour_[j]).norm()>.05f)break;
  }
  if(!changed)return false;
  path.swap(extended);
  ROS_INFO("Native TSP preview: first leg %.2fm, extended guide %.2fm",first_leg,used);
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
  nh.param("parallel_astar/goal_connection_timeout", goal_connection_timeout, .02);
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
  omp_set_num_threads(epic_ros::planningWorkerCount(4));
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
