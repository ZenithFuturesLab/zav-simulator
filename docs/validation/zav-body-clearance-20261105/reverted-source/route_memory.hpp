#pragma once
#include <super_utils/eigen_alias.hpp>
#include <algorithm>
#include <limits>

namespace path_search {
// Keep a geometric branch while it remains collision free. The caller still
// builds fresh corridors and optimizes/validates every executable trajectory.
class RouteMemory {
 public:
  using Path=super_utils::vec_Vec3f;
  using Point=super_utils::Vec3f;
  void clear(){path_.clear();}
  void remember(const Path&path,const Point&goal){path_=path;goal_=goal;}
  static double length(const Path&p){double d=0.;for(size_t i=1;i<p.size();++i)d+=(p[i]-p[i-1]).norm();return d;}
  template<class Safe>
  bool reuse(const Point&start,const Point&goal,double horizon,Safe safe,Path&out)const{
    out.clear();
    if(path_.size()<2||!start.allFinite()||!goal.allFinite()||!std::isfinite(horizon)||horizon<=0.||
       (goal-goal_).norm()>.05)return false;
    double best=std::numeric_limits<double>::infinity();size_t segment=0;Point projection;
    for(size_t i=0;i+1<path_.size();++i){
      Point d=path_[i+1]-path_[i];if(d.squaredNorm()<1e-12)continue;
      const double u=std::clamp((start-path_[i]).dot(d)/d.squaredNorm(),0.,1.);
      Point p=path_[i]+u*d;double distance=(p-start).squaredNorm();
      if(distance<best){best=distance;segment=i;projection=p;}
    }
    if(best>.75*.75)return false;
    // Join ahead along the retained branch, not back at its nearest old vertex.
    Point anchor=projection;double lookahead=std::min(.5,horizon);size_t next=segment+1;
    while(next<path_.size()){
      const double d=(path_[next]-anchor).norm();
      if(d>lookahead&&d>1e-9){anchor+=lookahead/d*(path_[next]-anchor);break;}
      anchor=path_[next++];lookahead-=d;
      if(lookahead<=1e-9)break;
    }
    Path suffix{start};if((anchor-start).norm()>1e-8)suffix.push_back(anchor);
    for(size_t i=next;i<path_.size();++i)if((path_[i]-suffix.back()).norm()>1e-8)suffix.push_back(path_[i]);
    out.push_back(start);double remaining=horizon;
    for(size_t i=1;i<suffix.size()&&remaining>1e-8;++i){
      Point end=suffix[i];const double d=(end-out.back()).norm();if(d<1e-9)continue;
      if(d>remaining)end=out.back()+remaining/d*(end-out.back());
      if(!safe(out.back(),end)){out.clear();return false;}
      remaining-=(end-out.back()).norm();out.push_back(end);
    }
    if(out.size()<2||(length(out)<.5&&(goal-start).norm()>1.)){out.clear();return false;}
    return true;
  }
 private:
  Path path_;Point goal_=Point::Zero();
};
}
