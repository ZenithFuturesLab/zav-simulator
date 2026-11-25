/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-07-12 12:22:06
 * @LastEditTime: 2024-08-05 20:08:38
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#pragma once
#include <optional>
#include "lidar_map/ikd_Tree.h"
#include <bits/stdc++.h>
#include <frontier_manager/raycast.h>
#include <lidar_map/lidar_map.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointcloud_topo/graph.h>
#include <epic_ros2/ros.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
using namespace fast_planner;

enum CELL_STATE { DENSE, SPARSE, UNKNOWN, FRONTIER_DIS, FRONTIER_DIR };

struct Vector3i_Hash {
  std::size_t operator()(const Eigen::Vector3i &v) const {
    std::size_t seed = 0;
    seed ^= std::hash<int>()(v.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(v.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(v.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct FrontierParam {
  float target_min_z_, target_max_z_, target_max_abs_normal_z_;
  float cluster_radius_;
  float cluster_min_radius_;
  float cluster_direction_radius_;
  int cluster_minmum_point_num_;
  float cell_size_;
  float inv_cell_size_;
  float occ_min_dis_;
  float good_observation_direction_score_;
  float good_observation_trust_length_;
  float update_length_;
  float good_observation_force_trust_length_;
  int dense_cell_cloud_num_;
  int sparse_cell_cloud_num_;
  int noise_cell_range_;
  float cluster_min_size_;
  Eigen::Vector3f map_min_;
  Eigen::Vector3f map_max_;
  Eigen::Vector3i cell_max_cnt_;
  Eigen::Vector3i bits_need_;
  uint8_t idx_byte_size_;
  bool view_cluster_, view_frt_;
};

struct ViewpointParam {
  int sample_pillar_circle_sample_num_, sample_pillar_radius_layer_num_,
      sample_pillar_height_layer_num_;
  float sample_pillar_min_height_, sample_pillar_max_height_,
      sample_pillar_min_radius_, sample_pillar_max_radius_,
      view_direction_range_;
  float unreachable_retry_interval_;
  bool fixed_height_enabled_;
  float fixed_height_, reached_position_tolerance_, reached_yaw_tolerance_;
  float fov_up_, fov_down_, lidar_pitch_;
  int local_tsp_size_;
};
struct ByteArrayRaw {
  uint8_t *data;
  static size_t size;

  ByteArrayRaw() { data = new uint8_t[size]; }

  ByteArrayRaw(const ByteArrayRaw &other) {
    data = new uint8_t[size];
    memcpy(data, other.data, size);
  }

  ByteArrayRaw &operator=(const ByteArrayRaw &other) {
    if (this != &other) {
      memcpy(data, other.data, size);
    }
    return *this;
  }

  ~ByteArrayRaw() { delete[] data; }

  bool operator==(const ByteArrayRaw &other) const {
    return memcmp(data, other.data, size) == 0;
  }
};

struct ByteArrayRawHasher {
  size_t operator()(const ByteArrayRaw &arr) const {
    const uint64_t seed = 0xc3a5c85c97cb3127ULL;
    const uint64_t m = 0xc6a4a7935bd1e995ULL;
    const int r = 47;

    size_t h = seed ^ (ByteArrayRaw::size * m);

    // 处理8字节对齐的数据
    const uint64_t *data = reinterpret_cast<const uint64_t *>(arr.data);
    const uint64_t *end = data + (ByteArrayRaw::size / 8);

    while (data != end) {
      uint64_t k = *data++;

      k *= m;
      k ^= k >> r;
      k *= m;

      h ^= k;
      h *= m;
    }

    // 处理剩余字节
    const unsigned char *tail = reinterpret_cast<const unsigned char *>(data);

    switch (ByteArrayRaw::size & 7) {
    case 7:
      h ^= uint64_t(tail[6]) << 48;
    case 6:
      h ^= uint64_t(tail[5]) << 40;
    case 5:
      h ^= uint64_t(tail[4]) << 32;
    case 4:
      h ^= uint64_t(tail[3]) << 24;
    case 3:
      h ^= uint64_t(tail[2]) << 16;
    case 2:
      h ^= uint64_t(tail[1]) << 8;
    case 1:
      h ^= uint64_t(tail[0]);
      h *= m;
    }

    h ^= h >> r;
    h *= m;
    h ^= h >> r;

    return h;
  }
};
struct FrontierData {
  FrontierData() { ByteArrayRaw::size = 8; }
  FrontierData(size_t idx_bytes_size) {
    ByteArrayRaw::size = idx_bytes_size; // 设置全局size
  }
  unordered_map<ByteArrayRaw, uint8_t, ByteArrayRawHasher> label_map_;
  unordered_map<ByteArrayRaw, Eigen::Vector3f, ByteArrayRawHasher> frt_map_;
  vector<bool> is_gap_;
  vector<float> direction_score_;
  vector<bool> is_fov_edge_;
  vector<float> sphere_distance_;
  Eigen::Vector3f updating_aabb_min;
  Eigen::Vector3f updating_aabb_max;
};

struct ViewpointCluster {
  ViewpointCluster() {
    center_ = Eigen::Vector3f::Zero();
    vps_.clear();
  }

  Eigen::Vector3f center_;
  PointVector vps_;
  float distance_;
};

struct ObservationSample {
  enum class Kind { UnknownAir, WallFace, AnyFace };
  Kind kind;
  Eigen::Vector3f point;
  Eigen::Vector3f toward_sensor=Eigen::Vector3f::Zero();
};

struct ClusterInfo {
  typedef shared_ptr<ClusterInfo> Ptr;
  // index of the frt
  PointVector cells_;
  vector<Eigen::Vector3f> norms_;
  Eigen::Vector3f center_;
  Eigen::Vector3f normal_;
  Eigen::Vector4f view_halfspace_;

  int id_;
  int odom_id_;
  vector<ViewpointCluster> vp_clusters_;
  Eigen::Vector3f best_vp_;
  PointVector visible_cells_;
  bool close_surface_sample_ = false;
  bool displaced_issued_goal_ = false;
  bool right_hand_opening_ = false;
  bool follows_local_outline_ = false;
  bool exterior_pass_ = false;
  bool aperture_approach_ = false;
  vector<Eigen::Vector3f> contour_approach_;
  vector<Eigen::Vector3f> contour_preview_;
  float fresh_contour_length_ = 0.f;
  std::vector<ObservationSample> contour_support_;
  std::optional<Eigen::Vector3f> contour_entry_;
  float best_vp_yaw_;
  float distance_;

  double last_viewpoint_attempt_ = -1.0e9;
  bool is_dormant_;
  bool is_reachable_;
  bool is_new_cluster_;

  // bbox
  Eigen::Vector3f box_max_;
  Eigen::Vector3f box_min_;
};

struct SuperClusterInfo {
  typedef shared_ptr<SuperClusterInfo> Ptr;
  vector<ClusterInfo::Ptr> sons;
  Eigen::Vector3f waypoint;
};

class FrontierManager {
private:
  friend struct SurfaceVisibilityRegression;
  friend struct FrontierClusteringRegression;
  PointVector unclustered_frts_;
  vector<Eigen::Vector3f> unclustered_norms_;
  FrontierParam frtp_;
  FrontierData frtd_;
  ViewpointParam vpp_;
  unordered_set<int> force_recluster_;
  LIOInterface::Ptr
      lidar_map_interface_; // TODO: 改成
                            // sim_lio,去掉不必要的接口,ikd-treeh和odom存储在里面
  TopoGraph::Ptr graph_;
  epic_ros::NodeHandle nh_;
  vector<Eigen::Vector3f> origin_viewpoints_;
  // frontier update functions:
  void update_lidar_pos();
  void update_updating_aabb(const PointVector &new_frt_pts);
  void cluster_frts(const PointVector &frt_new,
                    vector<ClusterInfo::Ptr> &new_clusters,
                    vector<int> &cluster_removed);
  void compute_cluster_info(const PointVector &frt_pts,
                            const vector<Eigen::Vector3f> &frt_norms,
                            ClusterInfo::Ptr cluster);
  bool has_overlap(const Eigen::Vector3f &box_max_,
                   const Eigen::Vector3f &box_min_);
  void project_pts_2_depth_image(PointVector &pts_vec,
                                 vector<float> &depth_img);
  int surface_pos2idx(const PointType &pt); // 2w个点
  Eigen::Vector3f map_min_bd_, map_max_bd_;
  void pos2idx(const PointType &pt, Eigen::Vector3i &idx);
  void pos2idx(const Eigen::Vector3f &pt, Eigen::Vector3i &idx);
  void pos2bytes(const PointType &pt, ByteArrayRaw &bytes);
  void bytes2pos(const ByteArrayRaw &bytes, PointType &pt);

  void idx2bytes(const Eigen::Vector3i &idx, ByteArrayRaw &bytes);
  void idx2pos(const Eigen::Vector3i &idx, PointType &pt);

  void update_lidar_pt_gap(const vector<float> &depth);
  void update_lidar_fov_edge(const vector<float> &depth);
  void computeNormal(const PointVector &local_pts, Eigen::Vector3f &normal);
  void computeNormalCell(const PointVector &local_pts, Eigen::Vector3f &normal,
                         Eigen::Vector3f &center);
  void Sphere_PosToIndex(const Eigen::Vector3f &lidar_center,
                         const Eigen::Vector3f &pos, Eigen::Vector2i &id);
  CELL_STATE get_state(const PointType &pt);
  CELL_STATE get_state(const Eigen::Vector3i &idx);
  bool is_gap_point(const PointType &pt);
  bool is_fov_edge(const PointType &pt);
  void get_cells_2_update(const PointVector &points,
                          vector<Eigen::Vector3i> &cells_2_update);
  void get_pts_in_cells(const vector<Eigen::Vector3i> &cells_2_update,
                        vector<PointVector> &pts_inside);
  void updateHalfSpaces(vector<ClusterInfo::Ptr> &clusters);
  //                                const vector<float> &bubble_radius);
  void selectBestViewpoint(ClusterInfo::Ptr &cluster);
  void initClusterViewpoints(ClusterInfo::Ptr &cluster);
  void removeUnreachableViewpoints(vector<ClusterInfo::Ptr> &clusters);
  bool isInBox(const PointType &pt);
  bool isInBox(const Eigen::Vector3f &pt);
  bool computeSuperClusterInfo(SuperClusterInfo::Ptr &super_cluster);
  void reclusterSuperCluster(SuperClusterInfo::Ptr &super_cluster);

public:
  typedef std::shared_ptr<FrontierManager> Ptr;
  Eigen::Isometry3f transform_world2lidar;

  void init(epic_ros::NodeHandle &nh, LIOInterface::Ptr &lio_interface,
            TopoGraph::Ptr graph);
  // unordered_map<int, ClusterInfo::Ptr> new_clusters_;
  std::list<ClusterInfo::Ptr> cluster_list_;
  bool hasInspectionFrontiers() const;
  bool exteriorPassPending() const {return perimeter_pass_ && !perimeter_pass_->finished;}
  bool hasPendingSurveyFrontiers(const Eigen::Vector3f &body);
  const std::optional<Eigen::Vector3f>& pendingSurveyLocation() const { return pending_survey_location_; }

  void printMemoryCost();

  void viz_pocc();
  void viz_point(PointVector &pts2viz, string topic_name);
  void viz_point(vector<Eigen::Vector3f> &pts2viz, string topic_name);
  void visfrtnorm(const std::vector<Eigen::Vector3f> &centers,
                  const std::vector<Eigen::Vector3f> &normals);
  void visfrtcluster();
  void vizBestViewpoint();
  void updateFrontierClusters(vector<ClusterInfo::Ptr> &cluster_updated,
                              vector<int> &cluster_removed);
  // Only candidate nodes from the current TSP solve own an observation.
  bool observedAir(const Eigen::Vector3f &point) const;
  bool connectObserved(const Eigen::Vector3f &start,const Eigen::Vector3f &goal,double budget,
                       vector<Eigen::Vector3f> &route) const;
  bool connectDistantObserved(const Eigen::Vector3f &start,const Eigen::Vector3f &goal,
                              vector<Eigen::Vector3f> &route,vector<Eigen::Vector3f> *prefix=nullptr) const;
  bool certifyFlightGuide(const vector<Eigen::Vector3f> &path) const;
  float measuredStanceClearance() const;
  bool withinFlightHeight(const Eigen::Vector3f &position) const;
  std::optional<Eigen::Vector2f> recentFlightDirection(const Eigen::Vector3f &body) const;
  std::optional<std::pair<Eigen::Vector3f,Eigen::Vector2f>> recentRightTurn(const Eigen::Vector3f &body) const;
  bool observedGuide(const vector<Eigen::Vector3f> &path) const;
  std::optional<float> survey_height_;
  double ground_clearance_=0.;
  bool hasLateralSurveyGain(const Eigen::Vector3f &stance,bool following_wall=false) const;
  bool aboveSameSurveyFloor(const Eigen::Vector3f &goal) const;
  bool exhaustedAboveFlownFloor(const vector<Eigen::Vector3f> &route) const;
  bool observedAboveFlownFloor(const ClusterInfo &cluster,float band) const;
  bool surveyedLowerSurface(const ClusterInfo &cluster,const Eigen::Vector3f &body) const;
  bool approachedClusterCells(const ClusterInfo &cluster,std::optional<float> survey_band={},
                              bool require_face=false) const;
  bool followsRisingFloor(const vector<Eigen::Vector3f> &route) const;
  std::optional<Eigen::Vector2f> surveyFlightDirection(const Eigen::Vector3f &body) const;
  std::optional<float> measuredSurveyHeight(const Eigen::Vector3f &body,float clearance) const;
  std::optional<float> currentSurveyBand(const Eigen::Vector3f &body) const;
  static bool withinLevelApproach(float height,float body_height,float survey_height,float radius);
  std::optional<Eigen::Vector3f> perimeter_target_;
  bool surface_continuity_ = false;
  bool observedCluster(const ClusterInfo &cluster) const;
  bool flownObservationStance(const Eigen::Vector3f &point) const;
  bool surveyedEnclosureStance(const Eigen::Vector3f &point) const;
  bool spentSurveyObservation(const ClusterInfo &observation,const Eigen::Vector3f &stance) const;
  void connectForwardAperture(const Eigen::Vector3f &body,const vector<TopoNode::Ptr> &viewpoints);
  bool reconnectSelectedApproach(const Eigen::Vector3f &body,const Eigen::Vector3f &goal,
                                 vector<Eigen::Vector3f> &route);
  bool shortenReturningPass(vector<Eigen::Vector3f> &guide) const;
  void confirmWallGuide(const vector<Eigen::Vector3f> &approach,const vector<Eigen::Vector3f> &preview,
                        bool has_entry=false);
  bool opposesIssuedRightTurn(const Eigen::Vector3f &body,const Eigen::Vector3f &proposal) const;
  enum class Departure { Backwards, Transfer, Forward, AlongWall, RightOpening };
  bool followsRightWall(const vector<Eigen::Vector3f> &guide) const;
  bool keepsRightWall(const vector<Eigen::Vector3f> &guide,float lookahead) const;
  vector<Eigen::Vector3f> previewRightWall(const Eigen::Vector3f &start,
                                         const Eigen::Vector2f &heading) const;
  Departure surveyDeparture(const Eigen::Vector3f &body,const Eigen::Vector2f &heading,
                            const vector<Eigen::Vector3f> &guide,bool wall,bool opening) const;
  TopoNode::Ptr continueIssuedRightTurn(const Eigen::Vector3f &body,const Eigen::Vector3f &proposal);
  bool reachableOpening(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route,
                        const Eigen::Vector3f *toward=nullptr);
  bool clearanceDepartureGuide(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route,
                               const Eigen::Vector3f *return_goal=nullptr);
  bool observedBoundaryGuide(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route,
                             const Eigen::Vector2f *forward=nullptr,const Eigen::Vector3f *toward=nullptr,
                             float minimum_forward=0.f);
  bool observedBoundaryGuide(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route,
                             const Eigen::Vector2f *forward,const Eigen::Vector3f *toward,
                             float minimum_forward,vector<Eigen::Vector3f> *prefix,std::optional<float> survey_level={},
                             std::optional<float> fork_distance={},bool ascending=false);
  bool revisitsLowerPass(const Eigen::Vector3f &body,const vector<Eigen::Vector3f> &route) const;
  bool risingBoundaryGuide(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route);
  bool continueBeforeLeftTurn(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route);
  bool pendingLevelGuide(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route);
  int unknownBoundaryFaces(const Eigen::Vector3i &key,vector<ObservationSample> *samples=nullptr);
  bool resumeSurveyThroughAperture(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route);
  bool previousOpening(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route);
  bool initialSurveyGuide(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route);
  bool sightlineGoal(const Eigen::Vector3f &body, vector<Eigen::Vector3f> &route);
  std::vector<Eigen::Vector3i> unseenSupport(const ClusterInfo &cluster) const;
  bool measureUnseenWall(const Eigen::Vector3f &stance, ClusterInfo &observation) const;
  bool hasRevisitableSurface(const ClusterInfo &observation,const Eigen::Vector3f &stance) const;
  bool visitedStance(const Eigen::Vector3f &point) const {
    return visited_air_.count((point/frtp_.cell_size_).array().floor().cast<int>())!=0;
  }
  float sensorUpdateRange() const { return frtp_.update_length_; }
  static size_t heightBoundedPrefix(const vector<Eigen::Vector3f> &path,double margin);
  float getCellSize() const { return frtp_.cell_size_; }
  std::unordered_map<const TopoNode*, ClusterInfo::Ptr> observations_;
private:
  std::unordered_set<Eigen::Vector3i,Vector3i_Hash> observed_air_;
  std::unordered_set<Eigen::Vector3i,Vector3i_Hash> visited_air_;
  // Upright-wall rasters consume only a horizontal slice of accumulated lidar hits.
  using HitLayer = std::unordered_set<Eigen::Vector3i,Vector3i_Hash>;
  std::unordered_map<int,HitLayer> measured_hits_by_layer_;
  void recordMeasuredHit(const Eigen::Vector3i &key) {
    measured_hits_by_layer_[key.z()].insert(key);
  }
  struct FaceEvidence {
    std::array<uint64_t,4> occupied_subcells{};
    std::array<uint64_t,4> first_scan{};
    std::array<uint64_t,4> all_subcells{},last_scan{};
    std::array<unsigned char,4> scan_count{};
  };
  std::unordered_map<Eigen::Vector3i,FaceEvidence,Vector3i_Hash> face_evidence_;
  std::unordered_map<Eigen::Vector3i,unsigned char,Vector3i_Hash> inspected_faces_;
  uint64_t surface_scan_=0;
  void recordFaceHit(const Eigen::Vector3f &sensor,const Eigen::Vector3f &hit);
  bool supportsElevatedOpening(const Eigen::Vector3f &goal) const;
  vector<Eigen::Vector3f> flight_history_, previous_outline_;
  std::optional<std::pair<Eigen::Vector3f,Eigen::Vector2f>> issued_right_turn_;
  std::optional<float> highest_flown_band_;
  std::optional<float> lowest_flown_band_;
  std::optional<float> retiredSurveyCeiling() const;
  struct PerimeterPass {
    float level=0.f;
    Eigen::Vector2f centre=Eigen::Vector2f::Zero(),start=Eigen::Vector2f::Zero(),last=Eigen::Vector2f::Zero();
    float angle=0.f;
    bool joined=false,finished=false;
  };
  std::optional<PerimeterPass> perimeter_pass_;
  TopoNode::Ptr exteriorViewpoint(const Eigen::Vector3f &body,bool observe_only=false);
  std::optional<Eigen::Vector3f> pending_survey_location_;
  vector<Eigen::Vector3f> branch_anchors_;
  size_t next_branch_anchor_=0;
  std::optional<std::pair<Eigen::Vector3f,size_t>> ascent_search_;
  bool untested_ascent_=false;
  std::optional<std::pair<Eigen::Vector3f,size_t>> residual_sweep_;
  bool connectObservedSections(const Eigen::Vector3f &start,const Eigen::Vector3f &goal,
                               vector<Eigen::Vector3f> &route,bool aperture=false) const;
  bool connectObservedGrid(const Eigen::Vector3f &start,const Eigen::Vector3f &goal,
                           float cell,double budget,vector<Eigen::Vector3f> &route,
                           bool distant=false) const;

  std::optional<Eigen::Vector3f> surface_body_;
  std::optional<std::pair<Eigen::Vector3f,double>> failed_centred_pass_;
  vector<ClusterInfo::Ptr> pending_surfaces_;
  size_t next_pending_surface_ = 0;
  std::optional<Eigen::Vector3f> sightline_target_;
  bool worthwhileRecoveryPass(const Eigen::Vector3f &body,const vector<Eigen::Vector3f> &departure) const;
  bool hasRemoteOpeningGain(const Eigen::Vector3f &body,const Eigen::Vector3f &stance) const;
  double sightline_selected_at_ = 0.;
  void expireOpeningAttempts(double now);
  void recordOpeningAttempt(const Eigen::Vector3i &key,double now);
  std::unordered_map<Eigen::Vector3i,double,Vector3i_Hash> attempted_sightlines_;
  double departure_margin_=-1.,aperture_clearance_=0.;
  vector<Eigen::Vector3f> last_aperture_path_;
  float rightWallSearchRange() const;
  std::optional<float> rightWallDistance(const Eigen::Vector3f &point,
      const Eigen::Vector2f &heading,float max_range) const;
  bool certifyCruiseGuide(const vector<Eigen::Vector3f> &path) const;
  bool certifyShortAperture(const vector<Eigen::Vector3f> &path) const;
public:
  bool surveyedFromAround(const Eigen::Vector3f &body,const Eigen::Vector3f &point) const;
  bool nearFlightPath(const Eigen::Vector3f &point,float radius,float ignore_recent=0.f,
                      float sight_clearance=0.f,float height_tolerance=std::numeric_limits<float>::infinity()) const;
  bool connectFlightHistory(const Eigen::Vector3f &start,const Eigen::Vector3f &goal,
                            double budget,vector<Eigen::Vector3f> &route,
                            vector<Eigen::Vector3f> *prefix=nullptr) const;
  bool prolongedRetrace() const { return retraced_distance_>4.f*frtp_.update_length_; }
  float retraced_distance_=0.f;
private:
  bool closesBoundarySweep(const vector<Eigen::Vector3f> &route) const;
  TopoNode::Ptr parallelObservation(const Eigen::Vector3f &body,bool require_separator=false);
  TopoNode::Ptr forwardObservation(const Eigen::Vector3f &body,bool require_broad=false);
  TopoNode::Ptr boundaryObservation(vector<Eigen::Vector3f> route);
  TopoNode::Ptr rollingSurveyView(const Eigen::Vector3f &body,const Eigen::Vector2f &direction);
  TopoNode::Ptr sampledSurveyPass(const Eigen::Vector3f &body,const PointVector &samples);
  TopoNode::Ptr surveyApproach(const Eigen::Vector3f &body,const Eigen::Vector3f &target);
  TopoNode::Ptr evaluateSurveyPath(const Eigen::Vector3f &body,const Eigen::Vector2f &along,
      vector<Eigen::Vector3f> route,float fresh_tail,bool centred,bool require_broad);
  bool centredForwardPass(const Eigen::Vector3f &body,const Eigen::Vector2f &along,
                          vector<Eigen::Vector3f> &path);
  void appendContourViewpoints(const Eigen::Vector3f &body, vector<TopoNode::Ptr> &viewpoints,bool include_alternative=false);
  void updateSurfaceSamples();
  void recordPassage(const Eigen::Vector3f &body);
  bool needsOppositeTransit(const Eigen::Vector3f &point,const Eigen::Vector2f &direction) const;
  bool observedFace(const Eigen::Vector3f &point,const Eigen::Vector3f &normal) const;
  std::optional<Eigen::Vector3f> preferRightBranch(const Eigen::Vector3f &body,vector<Eigen::Vector3f> &route,bool initial);
  TopoNode::Ptr initialBoundaryObservation(const Eigen::Vector3f &body,const TopoNode::Ptr &view);
  void appendSurfaceViewpoints(const Eigen::Vector3f &body, vector<TopoNode::Ptr> &viewpoints,bool include_alternative=false);
public:
  void publishSurveyViewpoints(const vector<TopoNode::Ptr> &viewpoints);
  void generateTSPViewpoints(Eigen::Vector3f &center_pose,
                             vector<TopoNode::Ptr> &viewpoints);
};
