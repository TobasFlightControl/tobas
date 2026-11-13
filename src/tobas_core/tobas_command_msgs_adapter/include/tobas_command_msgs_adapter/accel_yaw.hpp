// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>

#include <tobas_command_msgs/msg/accel_yaw.hpp>

namespace tobas_command_msgs
{
struct AccelYaw
{
  std_msgs::msg::Header header;
  tobas_command_msgs::msg::Priority priority;
  tobas::kdl::Vector accel;
  double yaw;

  using SharedPtr = std::shared_ptr<AccelYaw>;
  using ConstSharedPtr = std::shared_ptr<const AccelYaw>;
  using UniquePtr = std::unique_ptr<AccelYaw>;
  using ConstUniquePtr = std::unique_ptr<const AccelYaw>;
};
}  // namespace tobas_command_msgs

template <>
struct rclcpp::TypeAdapter<tobas_command_msgs::AccelYaw, tobas_command_msgs::msg::AccelYaw>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_command_msgs::AccelYaw;
  using ros_message_type = tobas_command_msgs::msg::AccelYaw;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.header = _src.header;
    _dst.priority = _src.priority;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(_src.accel, _dst.accel);
    _dst.yaw = _src.yaw;
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.header = _src.header;
    _dst.priority = _src.priority;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(_src.accel, _dst.accel);
    _dst.yaw = _src.yaw;
  }
};

namespace tobas_command_msgs
{
using AccelYawAdapter = rclcpp::TypeAdapter<tobas_command_msgs::AccelYaw, tobas_command_msgs::msg::AccelYaw>;
}  // namespace tobas_command_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_command_msgs::AccelYaw, tobas_command_msgs::msg::AccelYaw);
