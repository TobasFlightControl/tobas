// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/propulsion_system/ice_propulsion_system/moment_constant.hpp>

#include <tobas_drone_msgs/msg/vpp_moment_constant.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::VppMomentConstant, tobas_drone_msgs::msg::VppMomentConstant>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::VppMomentConstant;
  using ros_message_type = tobas_drone_msgs::msg::VppMomentConstant;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.a = _src.a;
    _dst.b = _src.b;
    _dst.c = _src.c;
    _dst.phi0 = _src.phi0;
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.a = _src.a;
    _dst.b = _src.b;
    _dst.c = _src.c;
    _dst.phi0 = _src.phi0;
  }
};

namespace tobas_drone_msgs
{
using VppMomentConstantAdapter =
  rclcpp::TypeAdapter<tobas::VppMomentConstant, tobas_drone_msgs::msg::VppMomentConstant>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::VppMomentConstant, tobas_drone_msgs::msg::VppMomentConstant);
