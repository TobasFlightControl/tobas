// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/joint.hpp>

#include <tobas_kdl_msgs/msg/joint.hpp>

#include "./vector.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::kdl::Joint, tobas_kdl_msgs::msg::Joint>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::kdl::Joint;
  using ros_message_type = tobas_kdl_msgs::msg::Joint;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.name = _src.name;
    _dst.type = static_cast<uint8_t>(_src.type);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(_src.origin, _dst.origin);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(_src.axis(), _dst.axis);
    _dst.damping = _src.damping;
    _dst.friction = _src.friction;
    _dst.lower_limit = _src.lower_limit;
    _dst.upper_limit = _src.upper_limit;
    _dst.max_effort = _src.max_effort;
    _dst.max_velocity = _src.max_velocity;
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.name = _src.name;
    _dst.type = static_cast<tobas::kdl::Joint::JointType>(_src.type);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(_src.origin, _dst.origin);

    tobas::kdl::Vector axis;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(_src.axis, axis);
    _dst.axis(axis);

    _dst.damping = _src.damping;
    _dst.friction = _src.friction;
    _dst.lower_limit = _src.lower_limit;
    _dst.upper_limit = _src.upper_limit;
    _dst.max_effort = _src.max_effort;
    _dst.max_velocity = _src.max_velocity;
  }
};

namespace tobas_kdl_msgs
{
using JointAdapter = rclcpp::TypeAdapter<tobas::kdl::Joint, tobas_kdl_msgs::msg::Joint>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::kdl::Joint, tobas_kdl_msgs::msg::Joint);
