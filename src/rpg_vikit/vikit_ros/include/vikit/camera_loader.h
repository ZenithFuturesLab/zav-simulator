#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <string>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/polynomial_camera.h>
#include <vikit/params_helper.h>
#include "rclcpp/rclcpp.hpp"

namespace vk {
namespace camera_loader {

/// Load from ROS Namespace
/// 相比ros，传入参数上多了一个node参数，调用要注意
bool loadFromRosNs(rclcpp::Node::SharedPtr node, const std::string& ns, vk::AbstractCamera*& cam);

bool loadFromRosNs(rclcpp::Node::SharedPtr node, const std::string& ns, std::vector<vk::AbstractCamera*>& cam_list);

} // namespace camera_loader
} // namespace vk

#endif // VIKIT_CAMERA_LOADER_H_