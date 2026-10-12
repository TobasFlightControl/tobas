// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/euler.hpp>

#include <tobas_command_msgs/msg/angle_throttle_vector.hpp>

namespace tobas_command_msgs
{
struct AngleThrottleVector
{
  std_msgs::msg::Header header;
  tobas_command_msgs::msg::Priority priority;
  tobas::kdl::Euler angle;
  double throttle;
  double thrust_angle;

  using SharedPtr = std::shared_ptr<AngleThrottleVector>;
  using ConstSharedPtr = std::shared_ptr<const AngleThrottleVector>;
  using UniquePtr = std::unique_ptr<AngleThrottleVector>;
  using ConstUniquePtr = std::unique_ptr<const AngleThrottleVector>;
};
}  // namespace tobas_command_msgs

template <>
struct rclcpp::TypeAdapter<tobas_command_msgs::AngleThrottleVector, tobas_command_msgs::msg::AngleThrottleVector>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_command_msgs::AngleThrottleVector;
  using ros_message_type = tobas_command_msgs::msg::AngleThrottleVector;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.header = _src.header;
    _dst.priority = _src.priority;
    tobas_kdl_msgs::EulerAdapter::convert_to_ros_message(_src.angle, _dst.angle);
    _dst.throttle = _src.throttle;
    _dst.thrust_angle = _src.thrust_angle;
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.header = _src.header;
    _dst.priority = _src.priority;
    tobas_kdl_msgs::EulerAdapter::convert_to_custom(_src.angle, _dst.angle);
    _dst.throttle = _src.throttle;
    _dst.thrust_angle = _src.thrust_angle;
  }
};

namespace tobas_command_msgs
{
using AngleThrottleVectorAdapter =
  rclcpp::TypeAdapter<tobas_command_msgs::AngleThrottleVector, tobas_command_msgs::msg::AngleThrottleVector>;
}  // namespace tobas_command_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_command_msgs::AngleThrottleVector,
  tobas_command_msgs::msg::AngleThrottleVector);
