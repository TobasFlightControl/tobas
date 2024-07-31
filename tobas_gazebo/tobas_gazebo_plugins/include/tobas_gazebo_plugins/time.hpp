#pragma once

#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>

namespace gazebo
{
rclcpp::Duration operator-(const common::Time& lhs, const rclcpp::Time& rhs);

rclcpp::Duration operator-(const rclcpp::Time& lhs, const common::Time& rhs);
}  // namespace gazebo
