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

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace vk {

inline bool hasParam(const std::string &name,
                     std::shared_ptr<rclcpp::Node> node) {
  return node->has_parameter(name);
}

template <typename T>
T getParam(const std::string &name, const T &defaultValue,
           std::shared_ptr<rclcpp::Node> node) {
  rclcpp::Parameter param;
  if (node->get_parameter(name, param)) {
    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Found parameter: " << name << ", value: " << param.get_value<T>());
    return param.get_value<T>();
  } else {
    RCLCPP_WARN_STREAM(node->get_logger(),
                       "Cannot find value for parameter: "
                           << name << ", assigning default: " << defaultValue);
    node->declare_parameter(name, defaultValue);
  }
  return defaultValue;
}

template <typename T>
T getParam(const std::string &name, std::shared_ptr<rclcpp::Node> node) {
  rclcpp::Parameter param;
  int i = 0;
  while (!node->get_parameter(name, param)) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Cannot find value for parameter: "
                                                << name << ", will try again.");
    if ((i++) >= 5)
      return T();
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Found parameter: " << name << ", value: " << param.get_value<T>());
  return param.get_value<T>();
}

} // namespace vk

#endif // ROS_PARAMS_HELPER_H_
