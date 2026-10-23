#include <vikit/camera_loader.h>

namespace vk {
namespace camera_loader {

/// Load from ROS Namespace
/// 相比ros，传入参数上多了一个node参数，调用要注意
bool loadFromRosNs(rclcpp::Node::SharedPtr node, const std::string& ns, vk::AbstractCamera*& cam)
{
  bool res = true;
  
  // 获取相机模型
  std::string cam_model = node->declare_parameter<std::string>(ns + ".cam_model");

  if(cam_model == "Ocam")
  {
    cam = new vk::OmniCamera(node->declare_parameter<std::string>(ns + ".cam_calib_file"));
  }
  else if(cam_model == "Pinhole")
  {
    cam = new vk::PinholeCamera(
        node->declare_parameter<int>(ns + ".cam_width"),
        node->declare_parameter<int>(ns + ".cam_height"),
        node->declare_parameter<double>(ns + ".scale", 1.0),
        node->declare_parameter<double>(ns + ".cam_fx"),
        node->declare_parameter<double>(ns + ".cam_fy"),
        node->declare_parameter<double>(ns + ".cam_cx"),
        node->declare_parameter<double>(ns + ".cam_cy"),
        node->declare_parameter<double>(ns + ".cam_d0", 0.0),
        node->declare_parameter<double>(ns + ".cam_d1", 0.0),
        node->declare_parameter<double>(ns + ".cam_d2", 0.0),
        node->declare_parameter<double>(ns + ".cam_d3", 0.0));
  }
  else if(cam_model == "EquidistantCamera")
  {
    cam = new vk::EquidistantCamera(
        node->declare_parameter<int>(ns + ".cam_width"),
        node->declare_parameter<int>(ns + ".cam_height"),
        node->declare_parameter<double>(ns + ".scale", 1.0),
        node->declare_parameter<double>(ns + ".cam_fx"),
        node->declare_parameter<double>(ns + ".cam_fy"),
        node->declare_parameter<double>(ns + ".cam_cx"),
        node->declare_parameter<double>(ns + ".cam_cy"),
        node->declare_parameter<double>(ns + ".k1", 0.0),
        node->declare_parameter<double>(ns + ".k2", 0.0),
        node->declare_parameter<double>(ns + ".k3", 0.0),
        node->declare_parameter<double>(ns + ".k4", 0.0));
  }
  else if(cam_model == "PolynomialCamera")
  {
    cam = new vk::PolynomialCamera(
        node->declare_parameter<int>(ns + "/cam_width"),
        node->declare_parameter<int>(ns + "/cam_height"),
        node->declare_parameter<double>(ns + "/cam_fx"),
        node->declare_parameter<double>(ns + "/cam_fy"),
        node->declare_parameter<double>(ns + "/cam_cx"),
        node->declare_parameter<double>(ns + "/cam_cy"),
        node->declare_parameter<double>(ns + "/cam_skew"),
        node->declare_parameter<double>(ns + "/k2", 0.0),
        node->declare_parameter<double>(ns + "/k3", 0.0),
        node->declare_parameter<double>(ns + "/k4", 0.0),
        node->declare_parameter<double>(ns + "/k5", 0.0),
        node->declare_parameter<double>(ns + "/k6", 0.0),
        node->declare_parameter<double>(ns + "/k7", 0.0));
  }
  else if(cam_model == "ATAN")
  {
    cam = new vk::ATANCamera(
        node->declare_parameter<int>(ns + "/cam_width"),
        node->declare_parameter<int>(ns + "/cam_height"),
        node->declare_parameter<double>(ns + "/cam_fx"),
        node->declare_parameter<double>(ns + "/cam_fy"),
        node->declare_parameter<double>(ns + "/cam_cx"),
        node->declare_parameter<double>(ns + "/cam_cy"),
        node->declare_parameter<double>(ns + "/cam_d0"));
  }
  else
  {
    cam = NULL;
    res = false;
  }
  return res;
}

bool loadFromRosNs(rclcpp::Node::SharedPtr node, const std::string& ns, std::vector<vk::AbstractCamera*>& cam_list)
{
  bool res = true;

  std::string cam_model = node->get_parameter(ns + ".cam_model").get_value<std::string>();
  int cam_num = node->get_parameter(ns + ".cam_num").get_value<int>();
  
  for (int i = 0; i < cam_num; i++)
  {
    std::string cam_ns = ns + ".cam_" + std::to_string(i);
    std::string cam_model = node->get_parameter(cam_ns + ".cam_model").get_value<std::string>();
    
    if(cam_model == "FishPoly")
    {
      cam_list.push_back(new vk::PolynomialCamera(
        node->declare_parameter<int>(cam_ns + ".image_width"),
        node->declare_parameter<int>(cam_ns + ".image_height"),
        node->declare_parameter<double>(cam_ns + ".A11"),  // cam_fx
        node->declare_parameter<double>(cam_ns + ".A22"),  // cam_fy
        node->declare_parameter<double>(cam_ns + ".u0"),  // cam_cx
        node->declare_parameter<double>(cam_ns + ".v0"),  // cam_cy
        node->declare_parameter<double>(cam_ns + ".A12"), // cam_skew
        node->declare_parameter<double>(cam_ns + ".k2", 0.0),
        node->declare_parameter<double>(cam_ns + ".k3", 0.0),
        node->declare_parameter<double>(cam_ns + ".k4", 0.0),
        node->declare_parameter<double>(cam_ns + ".k5", 0.0),
        node->declare_parameter<double>(cam_ns + ".k6", 0.0),
        node->declare_parameter<double>(cam_ns + ".k7", 0.0)));
    }
    else if(cam_model == "Pinhole")
    {
      cam_list.push_back(new vk::PinholeCamera(
          node->declare_parameter<int>(ns + ".cam_width"),
          node->declare_parameter<int>(ns + ".cam_height"),
          node->declare_parameter<double>(ns + ".scale", 1.0),
          node->declare_parameter<double>(ns + ".cam_fx"),
          node->declare_parameter<double>(ns + ".cam_fy"),
          node->declare_parameter<double>(ns + ".cam_cx"),
          node->declare_parameter<double>(ns + ".cam_cy"),
          node->declare_parameter<double>(ns + ".cam_d0", 0.0),
          node->declare_parameter<double>(ns + ".cam_d1", 0.0),
          node->declare_parameter<double>(ns + ".cam_d2", 0.0),
          node->declare_parameter<double>(ns + ".cam_d3", 0.0)));
    }
    else 
    {
      res = false;
    }
  }

  return res;
}

} // namespace camera_loader
} // namespace vk