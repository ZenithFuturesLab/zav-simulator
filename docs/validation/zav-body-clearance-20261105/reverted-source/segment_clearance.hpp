#pragma once
#include <rog_map_ros/rog_map_ros2.hpp>
#include <cmath>
#include <algorithm>

namespace path_search {
// Exact CIRI-style point/segment distance on raw occupied voxel centers.
inline bool segmentHasBodyClearance(const rog_map::ROGMapROS::Ptr&map,
 const super_utils::Vec3f&a,const super_utils::Vec3f&b,double radius){
 if(!a.allFinite()||!b.allFinite()||!std::isfinite(radius)||radius<=0.)return false;
 if(!map->insideLocalMap(a)||!map->insideLocalMap(b))return false;
 const auto&cfg=map->getMapConfig();
 if(std::min(a.z(),b.z())<cfg.virtual_ground_height+radius-1e-8 ||
    std::max(a.z(),b.z())>cfg.virtual_ceil_height-radius+1e-8)return false;
 // boxSearch excludes boundary grid indices; include one extra cell.
 const double padding=radius+cfg.resolution+1e-6;
 super_utils::vec_Vec3f occupied;
 map->boxSearch(a.cwiseMin(b)-super_utils::Vec3f::Constant(padding),
                a.cwiseMax(b)+super_utils::Vec3f::Constant(padding),rog_map::OCCUPIED,occupied);
 const super_utils::Vec3f d=b-a;const double denominator=d.squaredNorm();
 for(const auto&p:occupied){
  const double t=denominator<1e-20?0.:std::clamp((p-a).dot(d)/denominator,0.,1.);
  if((p-a-t*d).squaredNorm()<(radius-1e-7)*(radius-1e-7))return false;
 }
 return true;
}
}
