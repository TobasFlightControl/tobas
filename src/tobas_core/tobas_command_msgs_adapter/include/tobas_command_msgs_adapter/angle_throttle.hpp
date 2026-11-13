// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/euler.hpp>

#include <tobas_command_msgs/msg/angle_throttle.hpp>

namespace tobas_command_msgs
{
struct AngleThrottle
{
  std_msgs::msg::Header header;
  tobas_command_msgs::msg::Priority priority;
  tobas::kdl::Euler angle;
  double throttle;

  using SharedPtr = std::shared_ptr<AngleThrottle>;
  using ConstSharedPtr = std::shared_ptr<const AngleThrottle>;
  using UniquePtr = std::unique_ptr<AngleThrottle>;
  using ConstUniquePtr = std::unique_ptr<const AngleThrottle>;
};
}  // namespace tobas_command_msgs

template <>
struct rclcpp::TypeAdapter<tobas_command_msgs::AngleThrottle, tobas_command_msgs::msg::AngleThrottle>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_command_msgs::AngleThrottle;
  using ros_message_type = tobas_command_msgs::msg::AngleThrottle;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.header = _src.header;
    _dst.priority = _src.priority;
    tobas_kdl_msgs::EulerAdapter::convert_to_ros_message(_src.angle, _dst.angle);
    _dst.throttle = _src.throttle;
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.header = _src.header;
    _dst.priority = _src.priority;
    tobas_kdl_msgs::EulerAdapter::convert_to_custom(_src.angle, _dst.angle);
    _dst.throttle = _src.throttle;
  }
};

namespace tobas_command_msgs
{
using AngleThrottleAdapter =
  rclcpp::TypeAdapter<tobas_command_msgs::AngleThrottle, tobas_command_msgs::msg::AngleThrottle>;
}  // namespace tobas_command_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_command_msgs::AngleThrottle, tobas_command_msgs::msg::AngleThrottle);
