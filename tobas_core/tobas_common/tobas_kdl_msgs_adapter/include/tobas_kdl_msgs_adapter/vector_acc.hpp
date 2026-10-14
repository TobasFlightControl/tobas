// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/vector_acc.hpp>

#include <tobas_kdl_msgs/msg/vector_acc.hpp>

#include "./vector.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::kdl::VectorAcc, tobas_kdl_msgs::msg::VectorAcc>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::kdl::VectorAcc;
  using ros_message_type = tobas_kdl_msgs::msg::VectorAcc;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(_src.p, _dst.p);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(_src.v, _dst.v);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(_src.dv, _dst.dv);
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(_src.p, _dst.p);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(_src.v, _dst.v);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(_src.dv, _dst.dv);
  }
};

namespace tobas_kdl_msgs
{
using VectorAccAdapter = rclcpp::TypeAdapter<tobas::kdl::VectorAcc, tobas_kdl_msgs::msg::VectorAcc>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::kdl::VectorAcc, tobas_kdl_msgs::msg::VectorAcc);
