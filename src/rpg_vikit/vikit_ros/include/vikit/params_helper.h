/*
 * ros_params_helper.h
 *
 *  Created on: Feb 22, 2013
 *      Author: cforster
 *
 * from libpointmatcher_ros
 */

 #ifndef ROS_PARAMS_HELPER_H_
 #define ROS_PARAMS_HELPER_H_
 
 #include <string>
 #include <rclcpp/rclcpp.hpp>
 
 namespace vk {
 
 inline
 bool hasParam(const rclcpp::Node::SharedPtr & nh, const std::string& name)
 {
   return nh->has_parameter(name);
 }
 
 template<typename T>
 T getParam(const rclcpp::Node::SharedPtr & nh, const std::string& name, const T& defaultValue)
 {
   T v;
   if(nh->get_parameter(name, v))
   {
     RCLCPP_INFO(rclcpp::get_logger("ros_params_helper"), "Found parameter: {}, value: {}", name, v);
     return v;
   }
   else
   {
     RCLCPP_WARN(rclcpp::get_logger("ros_params_helper"), "Cannot find value for parameter: {}, assigning default: {}", name, defaultValue);
   }
   return defaultValue;
 }
 
 template<typename T>
 T getParam(const rclcpp::Node::SharedPtr &nh, const std::string& name)
 {
   T v;
   int i = 0;
   while(nh->get_parameter(name, v) == false)
   {
     RCLCPP_ERROR(rclcpp::get_logger("ros_params_helper"), "Cannot find value for parameter: {}, will try again.", name);
     if ((i++) >= 5) return T();
   }
 
   RCLCPP_INFO(rclcpp::get_logger("ros_params_helper"), "Found parameter: {}, value: {}", name, v);
   return v;
 }
 
 } // namespace vk
 
 #endif // ROS_PARAMS_HELPER_H_