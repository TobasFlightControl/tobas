// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/propulsion_system/ice_propulsion_system/motor_constant.hpp>

#include <tobas_drone_msgs/msg/vpp_motor_constant.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::VppMotorConstant, tobas_drone_msgs::msg::VppMotorConstant>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::VppMotorConstant;
  using ros_message_type = tobas_drone_msgs::msg::VppMotorConstant;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.c0 = src.c0;
    dst.c1 = src.c1;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.c0 = src.c0;
    dst.c1 = src.c1;
  }
};

namespace tobas_drone_msgs
{
using VppMotorConstantAdapter = rclcpp::TypeAdapter<tobas::VppMotorConstant, tobas_drone_msgs::msg::VppMotorConstant>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::VppMotorConstant, tobas_drone_msgs::msg::VppMotorConstant);
