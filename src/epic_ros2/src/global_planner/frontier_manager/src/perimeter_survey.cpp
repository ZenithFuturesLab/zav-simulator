#include <frontier_manager/frontier_manager.h>
#include <opencv2/imgproc.hpp>

namespace {
using V2=Eigen::Vector2f;
using V3=Eigen::Vector3f;
struct Projection {
  V2 tangent;
  float arc,depth;
};
struct Boundary {
  std::vector<V2> vertices;
  std::vector<float> cumulative;
  std::vector<bool> supported;
  Projection project(const V2 &point) const {
    Projection best{V2::UnitX(),0.f,std::numeric_limits<float>::infinity()};
    for (size_t i=0;i<vertices.size();++i) {
      if (!supported.empty() && !supported[i]) continue;
      const V2 &start=vertices[i];
      const V2 delta=vertices[(i+1)%vertices.size()]-start;
      const float length=cumulative[i+1]-cumulative[i];
      if (length<.01f) continue;
      const V2 tangent=delta/length;
      const float along=std::clamp((point-start).dot(tangent),0.f,length);
      const float distance=(point-start-along*tangent).norm();
      if (distance<best.depth) best={tangent,cumulative[i]+along,distance};
    }
    return best;
  }
};
}

// Follow the outside before inspecting separate obstacles in a broad free
// component. Derive the envelope from the current floor's upright lidar
// returns. The convex envelope orders headings; only measured free air, a
// measured right face and a collision-checked connector may produce a goal.
TopoNode::Ptr FrontierManager::exteriorViewpoint(const V3 &body,bool observe_only) {
  bool directional_priority;
  nh_.param("global_planning/right_hand_priority",directional_priority,false);
  if (directional_priority ||
      ground_clearance_<=0. || measured_hits_by_layer_.empty() || !observedAir(body)) return {};
  const float cell=frtp_.cell_size_,radius=graph_->bubble_min_radius_;
  const float trust=frtp_.good_observation_trust_length_,range=frtp_.update_length_;
  if (perimeter_pass_ && std::abs(perimeter_pass_->level-body.z())>2.5f*radius)
    perimeter_pass_.reset();
  if (perimeter_pass_ && perimeter_pass_->finished) return {};
  const float z=body.z()+std::clamp(survey_height_.value_or(body.z())-body.z(),-cell,cell);
  const int half=int(std::ceil(4.f*range/cell)),side=2*half+1;
  const Eigen::Vector2i origin=(body.head<2>()/cell).array().floor().cast<int>().matrix()-Eigen::Vector2i::Constant(half);
  auto xy=[&](int x,int y)->V2{return cell*(origin.cast<float>()+V2(x+.5f,y+.5f));};
  const auto pixel=[&](const V2 &p)->cv::Point{
    const auto ij=((p/cell).array().floor().cast<int>().matrix()-origin).eval();
    return {ij.x(),ij.y()};
  };
  const cv::Rect bounds(0,0,side,side);
  cv::Mat below=cv::Mat::zeros(side,side,CV_8U),above=below.clone(),air=below.clone();
  for (const auto &[layer, hits] : measured_hits_by_layer_) {
    const float dz=(layer+.5f)*(.5f*cell)-z;
    if (std::abs(dz)>.8f*radius || std::abs(dz)<.2f*cell) continue;
    auto &slice=dz<0.f ? below : above;
    for (const auto &hit : hits) {
      const V2 point=(hit.head<2>().cast<float>().array()+.5f)*(.5f*cell);
      const auto ij=pixel(point);
      if (bounds.contains(ij)) slice.at<uchar>(ij)=1;
    }
  }
  if (!perimeter_pass_) {
    // The first tour may see different subsets of the spinning scan. Build
    // upright faces at the current height from the local accumulated returns.
    PointVector returns;
    const V3 centre(body.x(),body.y(),z),span(range,range,.8f*radius);
    lidar_map_interface_->boxSearch(centre-span,centre+span,returns);
    for (const auto &hit:returns) {
      const V3 point=hit.getVector3fMap();
      const float dz=point.z()-z;
      if (std::abs(dz)<.2f*cell || (point-centre).head<2>().squaredNorm()>range*range) continue;
      const auto index=pixel(point.head<2>());
      if (bounds.contains(index)) (dz<0.f?below:above).at<uchar>(index)=1;
    }
  }
  const cv::Mat kernel=cv::getStructuringElement(cv::MORPH_ELLIPSE,{3,3});
  cv::Mat high,low;cv::dilate(above,high,kernel);cv::dilate(below,low,kernel);
  const cv::Mat walls=(above&low)|(below&high);
  const auto observedPolygon=[&](const cv::Mat &wall_raster)->vector<cv::Point> {
    vector<cv::Point> points,hull,polygon;cv::findNonZero(wall_raster,points);
    if (points.size()<3) return {};
    cv::convexHull(points,hull,false);
    cv::approxPolyDP(hull,polygon,radius/cell,true);
    if (polygon.size()<3) return {};
    // Long measured sides provide stable tangents through a temporary lidar
    // shadow. Use their fitted rectangle only when three actual sides support it.
    cv::Point2f corners[4];cv::minAreaRect(points).points(corners);
    int supported_sides=0;
    for (int i=0;i<4;++i) {
      const cv::Point2f edge=corners[(i+1)%4]-corners[i];
      const float len=cv::norm(edge);if (len<1.f) continue;
      std::set<int> bins;
      for (const auto &pt:points) {
        const cv::Point2f d=cv::Point2f(pt)-corners[i];
        if (std::abs(d.cross(edge))/len*cell>radius) continue;
        const float along=d.dot(edge)/len;
        if (along>=0.f && along<=len) bins.insert(int(along));
      }
      if (bins.size()*cell>=std::min(2.f*trust,.55f*cell*len)) ++supported_sides;
    }
    if (supported_sides>=3) {
      polygon.clear();
      for (const auto &corner:corners) polygon.emplace_back(cvRound(corner.x),cvRound(corner.y));
    }
    if (cv::contourArea(polygon,true)<0.) std::reverse(polygon.begin(),polygon.end());
    return polygon;
  };
  auto polygon=observedPolygon(walls);
  if (polygon.empty()) return {};
  cv::Mat envelope_hits=walls;
  if (perimeter_pass_ && !observe_only) {
    // Keep the launch test local. Once it succeeds, use accumulated upright
    // returns to maintain the same exterior sides across individual scans.
    cv::Mat far_low=below.clone(),far_high=above.clone();
    PointVector returns;
    const V3 span(4.f*range,4.f*range,.8f*radius);
    const V3 centre(body.x(),body.y(),z);
    lidar_map_interface_->boxSearch(centre-span,centre+span,returns);
    for (const auto &hit:returns) {
      const V3 point=hit.getVector3fMap();
      const float dz=point.z()-z;
      if (std::abs(dz)<.2f*cell) continue;
      const auto index=pixel(point.head<2>());
      if (bounds.contains(index)) (dz<0.f?far_low:far_high).at<uchar>(index)=1;
    }
    cv::Mat low_near,high_near;
    cv::dilate(far_low,low_near,kernel);
    cv::dilate(far_high,high_near,kernel);
    envelope_hits=(far_low&high_near)|(far_high&low_near);
    auto measured=observedPolygon(envelope_hits);
    if (!measured.empty()) polygon.swap(measured);
  }
  Boundary boundary;boundary.cumulative.push_back(0.f);
  for (auto &p:polygon) boundary.vertices.push_back(xy(p.x,p.y));
  for (size_t i=0;i<polygon.size();++i)
    boundary.cumulative.push_back(boundary.cumulative.back()+
        (boundary.vertices[(i+1)%polygon.size()]-boundary.vertices[i]).norm());
  if (!observe_only) {
    // A hull can bridge free space between distant lidar returns. Only use
    // a side for steering when its measured upright surface covers it.
    cv::Mat distance;
    cv::distanceTransform(1-envelope_hits,distance,cv::DIST_L2,cv::DIST_MASK_PRECISE);
    for (size_t i=0;i<boundary.vertices.size();++i) {
      const V2 start=boundary.vertices[i];
      const V2 edge=boundary.vertices[(i+1)%boundary.vertices.size()]-start;
      const int steps=std::max(3,int(std::ceil(edge.norm()/cell)));
      int hits=0,run=0,longest=0;
      for (int j=1;j<steps;++j) {
        const auto index=pixel(start+edge*(float(j)/steps));
        if (bounds.contains(index) && cell*distance.at<float>(index)<radius) {
          ++hits;
          longest=std::max(longest,++run);
        } else {
          run=0;
        }
      }
      // A turn exposes only the near end of a long wall. A continuous
      // measured stretch can establish its tangent before its far end appears.
      boundary.supported.push_back(2*hits>=steps-1 || longest*cell>=2.f*trust);
    }
    if (std::none_of(boundary.supported.begin(),boundary.supported.end(),
                     [](bool supported){return supported;})) return {};
  }
  const int layer=int(std::floor(z/cell));
  for (const auto &k:observed_air_) {
    if (std::abs(k.z()-layer)>1) continue;
    const cv::Point ij(k.x()-origin.x(),k.y()-origin.y());
    if (bounds.contains(ij)) air.at<uchar>(ij)=1;
  }
  cv::Mat clear;cv::distanceTransform(1-walls,clear,cv::DIST_L2,cv::DIST_MASK_PRECISE);
  cv::Mat free;cv::morphologyEx(air,free,cv::MORPH_CLOSE,kernel);
  free.setTo(0,clear<measuredStanceClearance()/cell);
  if (!perimeter_pass_) {
    cv::Mat room;cv::distanceTransform(free,room,cv::DIST_L2,cv::DIST_MASK_PRECISE);
    if (flight_history_.empty()) return {};
    // An exterior-first circuit belongs to the same broad floor space as
    // takeoff. A large room reached through a narrow door is a local pass.
    cv::Mat broad;cv::connectedComponents(room>2.5f*radius/cell,broad);
    const auto launch=pixel(flight_history_.front().head<2>());
    if (!bounds.contains(launch) || !broad.at<int>(launch)) return {};
    double open_radius;
    cv::minMaxLoc(room,nullptr,&open_radius,nullptr,nullptr,broad==broad.at<int>(launch));
    if (cell*open_radius<1.5f*trust) return {};
    vector<vector<cv::Point>> loops;vector<cv::Vec4i> hierarchy;
    cv::findContours(free,loops,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_SIMPLE);
    // Walls separating two rooms can be visible from both sides. Require a
    // distinct interior footprint as well as a wide certified open component.
    cv::Mat islands,labels,stats,centres;cv::dilate(walls,islands,kernel);
    const int groups=cv::connectedComponentsWithStats(islands,labels,stats,centres);
    bool detached=false;
    for (int i=1;i<groups && !detached;++i) {
      const cv::Rect box(stats.at<int>(i,cv::CC_STAT_LEFT),stats.at<int>(i,cv::CC_STAT_TOP),
                         stats.at<int>(i,cv::CC_STAT_WIDTH),stats.at<int>(i,cv::CC_STAT_HEIGHT));
      if (cell*std::max(box.width,box.height)<3.f*radius) continue;
      const auto centre=cv::Point2f(centres.at<double>(i,0),centres.at<double>(i,1));
      if (cell*cv::pointPolygonTest(polygon,centre,true)<3.f*radius) continue;
      bool touches=false;
      for (int y=box.y;y<box.br().y && !touches;++y) for (int x=box.x;x<box.br().x && !touches;++x)
        if (labels.at<int>(y,x)==i && cell*cv::pointPolygonTest(polygon,cv::Point2f(x,y),true)<2.f*radius) touches=true;
      if (touches) continue;
      unsigned bearings=0;
      const int margin=int(std::ceil(2.f*radius/cell));
      const cv::Rect apron=(box+cv::Size(2*margin,2*margin)-cv::Point(margin,margin))&bounds;
      for (int y=apron.y;y<apron.br().y;++y) for (int x=apron.x;x<apron.br().x;++x)
        if (free.at<uchar>(y,x)) {
          const int bin=int((std::atan2(y-centre.y,x-centre.x)+M_PI)*4./M_PI)%8;
          bearings|=1u<<bin;
        }
      if (std::bitset<8>(bearings).count()<6) continue;
      // The measured free space must actually wrap around the obstacle.
      // Returns on both sides of a partition can otherwise resemble a rack.
      for (size_t j=0;j<loops.size() && !detached;++j)
        detached=hierarchy[j][3]>=0 && cv::pointPolygonTest(loops[j],centre,false)>0;
    }
    if (!detached) return {};
    const auto moment=cv::moments(polygon);if (std::abs(moment.m00)<1.) return {};
    PerimeterPass pass;pass.level=z;
    pass.centre=cell*(origin.cast<float>()+V2(moment.m10/moment.m00+.5,moment.m01/moment.m00+.5));
    perimeter_pass_=pass;
    ROS_INFO("Detected broad launch enclosure at %.2f %.2f %.2f from connected observed air",body.x(),body.y(),z);
  }
  if (observe_only) return {};
  auto &pass=*perimeter_pass_;
  const V2 here=body.head<2>();
  const auto departure=boundary.project(here);
  const auto hasRightFace=[&](const V2 &where,const V2 &tangent) {
    const V2 right(tangent.y(),-tangent.x());
    for (float r=cell;r<=3.5f*radius;r+=.5f*cell) {
      const auto p=pixel(where+r*right);
      if (bounds.contains(p) && walls.at<uchar>(p)) return true;
    }
    return false;
  };
  if (!pass.joined && departure.depth<3.f*radius && hasRightFace(here,departure.tangent) &&
      hasLateralSurveyGain(body,true)) {
    const auto centre=cv::moments(polygon);
    if (std::abs(centre.m00)>1.)
      pass.centre=cell*(origin.cast<float>()+V2(centre.m10/centre.m00+.5,centre.m01/centre.m00+.5));
    pass.joined=true;pass.start=pass.last=here;
  } else if (pass.joined && (here-pass.last).norm()>.2f*cell) {
    const V2 a=pass.last-pass.centre,b=here-pass.centre;
    pass.angle+=std::atan2(a.x()*b.y()-a.y()*b.x(),a.dot(b));pass.last=here;
    // An early scan can expose the outside wall after the vehicle has already
    // followed it. Close against that actual flown segment when the recorded
    // loop surrounds most of the measured floor envelope in the right direction.
    const auto closedAgainstHistory=[&] {
      if (pass.angle<float(M_PI) || departure.depth>4.f*radius || !hasRightFace(here,departure.tangent)) return false;
      const float area=cv::contourArea(polygon)*cell*cell;
      V2 next=V2::Zero();
      float twice_area=0.f;
      for (auto it=flight_history_.rbegin();it!=flight_history_.rend();++it) {
        if (std::abs(it->z()-pass.level)>radius) break;
        const V2 previous=it->head<2>()-here;
        twice_area+=previous.x()*next.y()-previous.y()*next.x();
        if (previous.norm()<3.f*radius && twice_area>1.1f*area &&
            (next-previous).dot(departure.tangent)>0.f) return true;
        next=previous;
      }
      return false;
    };
    if ((pass.angle>1.8f*float(M_PI) && (here-pass.start).norm()<3.f*radius) || closedAgainstHistory()) {
      pass.finished=true;
      ROS_INFO("Completed measured exterior circuit at %.2f %.2f %.2f",body.x(),body.y(),body.z());
      return {};
    }
  }

  const float grid=.5f*cell;
  const int span=int(std::ceil(1.5f*range/grid))+2,width=2*span+1,n=width*width;
  const cv::Rect search_bounds(0,0,width,width);
  const float inf=std::numeric_limits<float>::infinity();
  vector<float> cost(n,inf),length(n,0.f),clearance(n,-1.f);
  vector<int> parent(n,-1);
  vector<V3> poses(n);
  const auto location=[&](int k)->V2{return here+grid*V2(k%width-span,k/width-span);};
  const auto canFly=[&](int k) {
    if (clearance[k]>=0.f) return clearance[k]>=measuredStanceClearance();
    const V2 point=location(k);
    clearance[k]=0.f;
    const auto ij=pixel(point);
    if (!bounds.contains(ij) || !air.at<uchar>(ij)) return false;
    // A low lip can block one horizontal slice of a flyable passage. Compare
    // neighbouring observed heights within the same survey voxel.
    for (float dz:{0.f,grid,-grid,cell,-cell}) {
      const V3 q(point.x(),point.y(),z+dz);
      if (!withinFlightHeight(q) || !observedAir(q)) continue;
      const float d=lidar_map_interface_->getDisToOcc(q);
      if (d>clearance[k]) {clearance[k]=d;poses[k]=q;}
      if (d>=measuredStanceClearance()+.4f*cell) break;
    }
    return clearance[k]>=measuredStanceClearance();
  };
  vector<Projection> locations(n,{V2::Zero(),0.f,-1.f});
  const auto locate=[&](int k)->const Projection&{
    if (locations[k].depth<0.f) locations[k]=boundary.project(location(k));
    return locations[k];
  };
  using Item=std::pair<float,int>;std::priority_queue<Item,vector<Item>,std::greater<Item>> queue;
  const int start_id=span*width+span;
  for (int dy=-3;dy<=3;++dy) for (int dx=-3;dx<=3;++dx) {
    const int k=start_id+dy*width+dx;
    if (!canFly(k) || !certifyFlightGuide({body,poses[k]})) continue;
    length[k]=cost[k]=(poses[k]-body).norm();queue.emplace(cost[k],k);
  }
  vector<std::pair<float,int>> candidates;
  while (!queue.empty()) {
    const auto [score,k]=queue.top();queue.pop();if (cost[k]!=score) continue;
    const auto &at=locate(k);float progress=at.arc-departure.arc;
    if (progress<-.5f*boundary.cumulative.back()) progress+=boundary.cumulative.back();
    if (progress>1.5f*radius && progress<1.5f*range && at.depth<trust+cell &&
        clearance[k]<3.5f*radius && hasRightFace(location(k),at.tangent)) {
      const float benefit=std::min(progress,range)-.8f*std::max(0.f,progress-range);
      candidates.emplace_back(benefit-.7f*std::abs(at.depth-1.8f*radius)-.10f*cost[k]-
          .18f*std::max(0.f,length[k]-progress),k);
    }
    for (int dy=-1;dy<=1;++dy) for (int dx=-1;dx<=1;++dx) {
      if (!dx && !dy) continue;
      const cv::Point q(k%width+dx,k/width+dy);if (!search_bounds.contains(q)) continue;
      const int next=q.y*width+q.x;
      if (locate(next).depth>std::max(departure.depth+radius,2.f*trust) || !canFly(next)) continue;
      const float dz=std::abs(poses[k].z()-poses[next].z());
      if (dz>grid+.01f) continue;
      const float step=(poses[next]-poses[k]).norm(),travel=length[k]+step;
      if (travel>1.5f*range) continue;
      const float inset=std::max(0.f,locate(next).depth-2.f*radius);
      const float margin=std::max(0.f,1.8f*radius-clearance[next]);
      const float weight=score+step*(1.f+.22f*std::min(25.f,inset*inset)+2.f*margin*margin)+dz;
      if (weight>=cost[next]) continue;
      cost[next]=weight;length[next]=travel;parent[next]=k;queue.emplace(weight,next);
    }
  }
  std::sort(candidates.rbegin(),candidates.rend());
  vector<bool> blocked_edge(n,false);
  for (const auto &[score,id]:candidates) {
    const V3 goal=poses[id];
    // Failed gain at one cell says nothing about its neighbour across a
    // doorway. Only a failed edge is shared by its descendants in this tree.
    bool blocked=false;
    for (int k=id;k>=0;k=parent[k]) {
      if (blocked_edge[k]) {blocked=true;break;}
    }
    if (blocked || !hasLateralSurveyGain(goal,true)) continue;
    const auto &at=locate(id);
    vector<V3> raw;for(int k=id;k>=0;k=parent[k]) raw.push_back(poses[k]);
    raw.push_back(body);std::reverse(raw.begin(),raw.end());
    if (!certifyFlightGuide(raw)) {
      for (int k=id;parent[k]>=0;k=parent[k]) {
        if (!certifyFlightGuide({poses[parent[k]],poses[k]})) {
          blocked_edge[k]=true;
          break;
        }
      }
      continue;
    }
    vector<V3> guide{body};
    for (size_t i=0;i+1<raw.size();) {
      size_t j=i+1;
      for (;j+1<raw.size();++j) {
        const V3 edge=raw[j+1]-raw[i];bool near=true;
        for (size_t h=i+1;h<=j && near;++h) {
          const V3 delta=raw[h]-raw[i];
          near=(delta-std::clamp(delta.dot(edge)/std::max(.0001f,edge.squaredNorm()),0.f,1.f)*edge).norm()<radius;
        }
        if (!near || !certifyFlightGuide({raw[i],raw[j+1]})) break;
      }
      guide.push_back(raw[j]);i=j;
    }
    auto observation=std::make_shared<ClusterInfo>();observation->follows_local_outline_=true;
    observation->exterior_pass_=true;observation->close_surface_sample_=true;observation->contour_approach_=guide;
    observation->fresh_contour_length_=length[id];observation->center_=goal;
    auto node=std::make_shared<TopoNode>();node->is_viewpoint_=true;node->center_=goal;
    node->yaw_=std::atan2(at.tangent.y(),at.tangent.x());observations_[node.get()]=observation;
    if (std::getenv("EPIC_TRACE_WALL_CHOICE"))
      ROS_INFO("Exterior observation at %.2f %.2f from %.2f %.2f, inset %.2f, measured angle %.2f",
               goal.x(),goal.y(),body.x(),body.y(),at.depth,pass.angle);
    return node;
  }
  return {};
}

// A remote viewpoint may be reachable through both ends of an obstacle.
// Replace a long retreat with a measured turn at the current end when the
// same viewpoint can be reached with less travel and full flight clearance.
bool FrontierManager::shortenReturningPass(vector<V3> &guide) const {
  if (!perimeter_pass_ || !perimeter_pass_->finished || guide.size()<2) return false;
  const V3 start=guide.front(),goal=guide.back();
  const auto heading=surveyFlightDirection(start);
  const float radius=graph_->bubble_min_radius_,trust=frtp_.good_observation_trust_length_;
  if (!heading || std::abs(start.z()-perimeter_pass_->level)>radius || std::abs(goal.z()-start.z())>radius) return false;
  const V2 right(heading->y(),-heading->x()),delta=(goal-start).head<2>();
  const float lateral=delta.dot(right),along=delta.dot(*heading);
  if (std::abs(lateral)<2.5f*radius || std::abs(lateral)>2.f*trust || along>-radius) return false;
  float length=0.f,furthest_forward=0.f,retreat=0.f;
  for (size_t i=1;i<guide.size();++i) {
    length+=(guide[i]-guide[i-1]).norm();
    const V2 offset=(guide[i]-start).head<2>();
    if (std::abs(offset.dot(right))<=2.f*radius) {
      furthest_forward=std::max(furthest_forward,offset.dot(*heading));
      retreat=std::max(retreat,-offset.dot(*heading));
    }
  }
  if (retreat<trust || furthest_forward>=radius) return false;
  // The selected viewpoint can be off-centre beside a shelf. Search a small
  // family of bend widths and join the exact observation from the exit lane.
  const float cell=frtp_.cell_size_;
  struct Bend {float length,lead,width;};
  vector<Bend> bends;
  for (float change=0.f;change<=2.f*radius;change+=cell) {
    for (int sign : {-1,1}) {
      if (change==0.f && sign>0) continue;
      const float width=std::abs(lateral)+sign*change;
      if (width<=2.f*radius) continue;
      for (float lead=0.f;lead<=1.5f*trust;lead+=cell) {
        const float exit_leg=std::hypot(lead-along,width-std::abs(lateral));
        const float distance=lead+float(M_PI)*.5f*width+exit_leg;
        if (distance+radius>=length) break;
        bends.push_back({distance,lead,width});
      }
    }
  }
  std::sort(bends.begin(),bends.end(),[](const auto &a,const auto &b){return a.length<b.length;});
  for (const auto &bend:bends) {
    const float turn_radius=.5f*bend.width;
    const int steps=std::max(4,int(std::ceil(float(M_PI)*turn_radius/(1.5f*cell))));
    vector<V3> turn{start};
    if (bend.lead>0.f) turn.push_back(start+bend.lead*V3(heading->x(),heading->y(),0.f));
    for (int j=1;j<=steps;++j) {
      const float angle=float(M_PI)*j/steps;
      V3 point=start;
      point.head<2>()+=(bend.lead+turn_radius*std::sin(angle))*(*heading)+
          std::copysign(turn_radius,lateral)*(1.f-std::cos(angle))*right;
      turn.push_back(point);
    }
    turn.push_back(goal);
    if (!certifyFlightGuide(turn)) continue;
    guide.swap(turn);
    ROS_INFO("Connected the selected observation around the nearer end of the measured obstacle: %.1fm vs %.1fm",bend.length,length);
    return true;
  }
  return false;
}
