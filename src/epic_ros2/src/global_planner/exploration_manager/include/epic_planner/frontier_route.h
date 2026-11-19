#pragma once
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fast_planner {
using FrontierRoute = std::vector<Eigen::Vector3f>;

// Rank ONLY the first native graph leg. Unchosen frontiers remain in the tour.
inline double frontierContinuationCost(const FrontierRoute &path,
                                      const Eigen::Vector3f &heading) {
  if(path.size()<2 || heading.head<2>().norm()<.1f)return 0.;
  Eigen::Vector3f point=path.front();float left=2.f;
  for(size_t i=1;i<path.size();++i) {
    const Eigen::Vector3f delta=path[i]-point;const float length=delta.norm();
    if(length<1.e-5f)continue;
    if(length>=left){point+=delta*(left/length);break;}
    point=path[i];left-=length;
  }
  const Eigen::Vector2f d=(point-path.front()).head<2>();
  if(d.norm()<.1f)return 0.;
  const double angle=std::remainder(std::atan2(d.y(),d.x())-
      std::atan2(heading.y(),heading.x()),2.*M_PI);
  const double turn=std::abs(angle);
  const int rank=turn<=M_PI/3. ? 0 : turn>=5.*M_PI/6. ? 3 : angle<0. ? 1 : 2;
  return rank*100.+turn;
}

// Remove viewpoint zigzags only within a small lateral tolerance, with monotone
// progress and the caller's unchanged obstacle check. Never shortcut a U-turn.
template<class Clear>
FrontierRoute simplifyFrontierRoute(const FrontierRoute &path,float deviation,Clear clear) {
  if(path.size()<3 || deviation<=0.f)return path;
  FrontierRoute out{path.front()};
  for(size_t i=0;i+1<path.size();) {
    size_t chosen=i+1;
    for(size_t j=path.size()-1;j>i+1;--j) {
      const Eigen::Vector3f delta=path[j]-path[i];const float length=delta.norm();
      if(length<.05f)continue;
      const Eigen::Vector3f axis=delta/length;float previous=0.f;bool ordered=true;
      for(size_t k=i+1;k<j;++k) {
        const float along=(path[k]-path[i]).dot(axis);
        if(along<previous-.05f || along<-.05f || along>length+.05f ||
            (path[k]-path[i]-axis*std::clamp(along,0.f,length)).norm()>deviation) {
          ordered=false;break;
        }
        previous=along;
      }
      if(ordered && clear(path[i],path[j])){chosen=j;break;}
    }
    out.push_back(path[chosen]);i=chosen;
  }
  return out;
}

// Advance by measured motion, not the closest point anywhere on a crossing
// route. This cursor is route progress, never a synthetic coverage ledger.
inline FrontierRoute remainingFrontierRoute(const FrontierRoute &path,
    const Eigen::Vector3f &position,const Eigen::Vector3f &previous_position,double &progress) {
  if(path.size()<2)return {};
  const double window=std::max(1.,2.*double((position-previous_position).norm())+.5);
  double arc=0.,best=std::numeric_limits<double>::infinity(),selected=progress;
  Eigen::Vector3f projection;size_t next=0;
  for(size_t i=1;i<path.size();++i) {
    const Eigen::Vector3f delta=path[i]-path[i-1];const double length=delta.norm();
    if(length<1.e-5)continue;
    if(arc+length>=progress && arc<=progress+window) {
      const double low=std::max(0.,progress-arc),high=std::min(length,progress+window-arc);
      const double along=std::clamp(double((position-path[i-1]).dot(delta))/length,low,high);
      const Eigen::Vector3f p=path[i-1]+delta*float(along/length);
      const double distance=(p-position).squaredNorm();
      if(distance<best-1.e-6){best=distance;selected=arc+along;projection=p;next=i;}
    }
    arc+=length;
  }
  if(!next || best>4.)return {};
  progress=selected;FrontierRoute out{position};
  if((projection-position).norm()>.01f)out.push_back(projection);
  for(size_t i=next;i<path.size();++i)if((out.back()-path[i]).norm()>.01f)out.push_back(path[i]);
  return out;
}
} // namespace fast_planner
