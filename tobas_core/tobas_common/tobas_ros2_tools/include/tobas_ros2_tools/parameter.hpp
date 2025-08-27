#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ros2
{
template <typename T>
void declareParam(rclcpp::Node::SharedPtr node, const std::string& name, const T& default_value, bool declared_ok = true)
{
  if (node->has_parameter(name)) {
    if (declared_ok) {
      return;
    }
    else {
      throw rclcpp::exceptions::ParameterAlreadyDeclaredException("Parameter \"" + name + "\" is already declared.");
    }
  }
  else {
    node->declare_parameter(name, default_value);
  }
}
}  // namespace ros2
