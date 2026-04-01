// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>

#include <tobas_command_msgs/msg/pos_vel_acc_pitch_yaw.hpp>

namespace tobas_command_msgs
{
struct PosVelAccPitchYaw
{
  std_msgs::msg::Header header;
  tobas_command_msgs::msg::Priority priority;
  tobas::kdl::Vector pos;
  tobas::kdl::Vector vel;
  tobas::kdl::Vector acc;
  double pitch;
  double yaw;

  using SharedPtr = std::shared_ptr<PosVelAccPitchYaw>;
  using ConstSharedPtr = std::shared_ptr<const PosVelAccPitchYaw>;
  using UniquePtr = std::unique_ptr<PosVelAccPitchYaw>;
  using ConstUniquePtr = std::unique_ptr<const PosVelAccPitchYaw>;
};
}  // namespace tobas_command_msgs

template <>
struct rclcpp::TypeAdapter<tobas_command_msgs::PosVelAccPitchYaw, tobas_command_msgs::msg::PosVelAccPitchYaw>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_command_msgs::PosVelAccPitchYaw;
  using ros_message_type = tobas_command_msgs::msg::PosVelAccPitchYaw;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    dst.priority = src.priority;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.pos, dst.pos);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.vel, dst.vel);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.acc, dst.acc);
    dst.pitch = src.pitch;
    dst.yaw = src.yaw;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    dst.priority = src.priority;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.pos, dst.pos);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.vel, dst.vel);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.acc, dst.acc);
    dst.pitch = src.pitch;
    dst.yaw = src.yaw;
  }
};

namespace tobas_command_msgs
{
using PosVelAccPitchYawAdapter =
  rclcpp::TypeAdapter<tobas_command_msgs::PosVelAccPitchYaw, tobas_command_msgs::msg::PosVelAccPitchYaw>;
}  // namespace tobas_command_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_command_msgs::PosVelAccPitchYaw,
  tobas_command_msgs::msg::PosVelAccPitchYaw);
