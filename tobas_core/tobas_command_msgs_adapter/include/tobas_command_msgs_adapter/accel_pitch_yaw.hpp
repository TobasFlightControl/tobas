// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>

#include <tobas_command_msgs/msg/accel_pitch_yaw.hpp>

namespace tobas_command_msgs
{
struct AccelPitchYaw
{
  std_msgs::msg::Header header;
  tobas_command_msgs::msg::Priority priority;
  tobas::kdl::Vector accel;
  double pitch;
  double yaw;

  using SharedPtr = std::shared_ptr<AccelPitchYaw>;
  using ConstSharedPtr = std::shared_ptr<const AccelPitchYaw>;
  using UniquePtr = std::unique_ptr<AccelPitchYaw>;
  using ConstUniquePtr = std::unique_ptr<const AccelPitchYaw>;
};
}  // namespace tobas_command_msgs

template <>
struct rclcpp::TypeAdapter<tobas_command_msgs::AccelPitchYaw, tobas_command_msgs::msg::AccelPitchYaw>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_command_msgs::AccelPitchYaw;
  using ros_message_type = tobas_command_msgs::msg::AccelPitchYaw;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    dst.priority = src.priority;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.accel, dst.accel);
    dst.pitch = src.pitch;
    dst.yaw = src.yaw;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    dst.priority = src.priority;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.accel, dst.accel);
    dst.pitch = src.pitch;
    dst.yaw = src.yaw;
  }
};

namespace tobas_command_msgs
{
using AccelPitchYawAdapter =
  rclcpp::TypeAdapter<tobas_command_msgs::AccelPitchYaw, tobas_command_msgs::msg::AccelPitchYaw>;
}  // namespace tobas_command_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_command_msgs::AccelPitchYaw, tobas_command_msgs::msg::AccelPitchYaw);
