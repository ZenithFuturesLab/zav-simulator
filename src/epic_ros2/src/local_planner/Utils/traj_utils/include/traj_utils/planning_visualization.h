#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <epic_ros2/ros.hpp>
#include <vector>
#include <string>
#include <visualization_msgs/msg/marker.hpp>

using std::vector;
using std::string;
namespace fast_planner {
class PlanningVisualization {
private:
  enum TRAJECTORY_PLANNING_ID {
    GOAL = 1,
    PATH = 200,
    BSPLINE = 300,
    BSPLINE_CTRL_PT = 400,
    POLY_TRAJ = 500
  };


  /* data */
  /* visib_pub is seperated from previous ones for different info */
  epic_ros::NodeHandle node;
  epic_ros::Publisher traj_pub_;       // 0
  epic_ros::Publisher topo_pub_;       // 1
  epic_ros::Publisher predict_pub_;    // 2
  epic_ros::Publisher visib_pub_;      // 3, visibility constraints
  epic_ros::Publisher frontier_pub_;   // 4, frontier searching
  epic_ros::Publisher yaw_pub_;        // 5, yaw trajectory
  epic_ros::Publisher viewpoint_pub_;  // 6, viewpoint planning
  vector<epic_ros::Publisher> pubs_;   //

  int last_topo_path1_num_;
  int last_topo_path2_num_;
  int last_bspline_phase1_num_;
  int last_bspline_phase2_num_;
  int last_frontier_num_;

public:
  PlanningVisualization(/* args */) {
  }
  ~PlanningVisualization() {
  }
  PlanningVisualization(epic_ros::NodeHandle& nh);

  // new interface
  void fillBasicInfo(visualization_msgs::msg::Marker& mk, const Eigen::Vector3d& scale,
                     const Eigen::Vector4d& color, const string& ns, const int& id, const int& shape);
  void fillGeometryInfo(visualization_msgs::msg::Marker& mk, const vector<Eigen::Vector3d>& list);
  void fillGeometryInfo(visualization_msgs::msg::Marker& mk, const vector<Eigen::Vector3d>& list1,
                        const vector<Eigen::Vector3d>& list2);

  void drawSpheres(const vector<Eigen::Vector3d>& list, const double& scale,
                   const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id);
  void drawCubes(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color,
                 const string& ns, const int& id, const int& pub_id);
  void drawLines(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                 const double& scale, const Eigen::Vector4d& color, const string& ns, const int& id,
                 const int& pub_id);
  void drawLines(const vector<Eigen::Vector3d>& list, const double& scale, const Eigen::Vector4d& color,
                 const string& ns, const int& id, const int& pub_id);
  void drawBox(const Eigen::Vector3d& center, const Eigen::Vector3d& scale, const Eigen::Vector4d& color,
               const string& ns, const int& id, const int& pub_id);

  // Deprecated
  // draw basic shapes
  void displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                         const Eigen::Vector4d& color, int id, int pub_id = 0);
  void displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                       const Eigen::Vector4d& color, int id, int pub_id = 0);
  void displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                       double line_width, const Eigen::Vector4d& color, int id, int pub_id = 0);
  // draw a piece-wise straight line path
  void drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution,
                         const Eigen::Vector4d& color, int id = 0);

  void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);

  Eigen::Vector4d getColor(const double& h, double alpha = 1.0);

  typedef std::shared_ptr<PlanningVisualization> Ptr;

  // SECTION developing
  void drawVisibConstraint(const Eigen::MatrixXd& ctrl_pts, const vector<Eigen::Vector3d>& block_pts);
  void drawFrontier(const vector<vector<Eigen::Vector3d>>& frontiers);
};
}  // namespace fast_planner
#endif