// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/twist.hpp>

#include <tobas_kdl_msgs/msg/twist.hpp>

#include "./vector.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::kdl::Twist, tobas_kdl_msgs::msg::Twist>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::kdl::Twist;
  using ros_message_type = tobas_kdl_msgs::msg::Twist;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(_src.vel, _dst.linear);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(_src.rot, _dst.angular);
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(_src.linear, _dst.vel);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(_src.angular, _dst.rot);
  }
};

namespace tobas_kdl_msgs
{
using TwistAdapter = rclcpp::TypeAdapter<tobas::kdl::Twist, tobas_kdl_msgs::msg::Twist>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::kdl::Twist, tobas_kdl_msgs::msg::Twist);
