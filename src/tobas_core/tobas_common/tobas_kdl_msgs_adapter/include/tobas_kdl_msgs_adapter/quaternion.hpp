// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/quaternion.hpp>

#include <tobas_kdl_msgs/msg/quaternion.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::kdl::Quaternion, tobas_kdl_msgs::msg::Quaternion>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::kdl::Quaternion;
  using ros_message_type = tobas_kdl_msgs::msg::Quaternion;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.x = _src.x;
    _dst.y = _src.y;
    _dst.z = _src.z;
    _dst.w = _src.w;
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.x = _src.x;
    _dst.y = _src.y;
    _dst.z = _src.z;
    _dst.w = _src.w;
  }
};

namespace tobas_kdl_msgs
{
using QuaternionAdapter = rclcpp::TypeAdapter<tobas::kdl::Quaternion, tobas_kdl_msgs::msg::Quaternion>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::kdl::Quaternion, tobas_kdl_msgs::msg::Quaternion);
