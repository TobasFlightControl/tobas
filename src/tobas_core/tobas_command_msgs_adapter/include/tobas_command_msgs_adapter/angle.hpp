// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/euler.hpp>

#include <tobas_command_msgs/msg/angle.hpp>

namespace tobas_command_msgs
{
struct Angle
{
  std_msgs::msg::Header header;
  tobas_command_msgs::msg::Priority priority;
  tobas::kdl::Euler angle;

  using SharedPtr = std::shared_ptr<Angle>;
  using ConstSharedPtr = std::shared_ptr<const Angle>;
  using UniquePtr = std::unique_ptr<Angle>;
  using ConstUniquePtr = std::unique_ptr<const Angle>;
};
}  // namespace tobas_command_msgs

template <>
struct rclcpp::TypeAdapter<tobas_command_msgs::Angle, tobas_command_msgs::msg::Angle>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_command_msgs::Angle;
  using ros_message_type = tobas_command_msgs::msg::Angle;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.header = _src.header;
    _dst.priority = _src.priority;
    tobas_kdl_msgs::EulerAdapter::convert_to_ros_message(_src.angle, _dst.angle);
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.header = _src.header;
    _dst.priority = _src.priority;
    tobas_kdl_msgs::EulerAdapter::convert_to_custom(_src.angle, _dst.angle);
  }
};

namespace tobas_command_msgs
{
using AngleAdapter = rclcpp::TypeAdapter<tobas_command_msgs::Angle, tobas_command_msgs::msg::Angle>;
}  // namespace tobas_command_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_command_msgs::Angle, tobas_command_msgs::msg::Angle);
