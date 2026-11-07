// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/vector.hpp>

#include <tobas_kdl_msgs/msg/vector_stamped.hpp>

#include "./vector.hpp"

namespace tobas_kdl_msgs
{
struct VectorStamped
{
  std_msgs::msg::Header header;
  tobas::kdl::Vector vector;

  using SharedPtr = std::shared_ptr<VectorStamped>;
  using ConstSharedPtr = std::shared_ptr<const VectorStamped>;
  using UniquePtr = std::unique_ptr<VectorStamped>;
  using ConstUniquePtr = std::unique_ptr<const VectorStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::VectorStamped, tobas_kdl_msgs::msg::VectorStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::VectorStamped;
  using ros_message_type = tobas_kdl_msgs::msg::VectorStamped;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.header = _src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(_src.vector, _dst.vector);
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.header = _src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(_src.vector, _dst.vector);
  }
};

namespace tobas_kdl_msgs
{
using VectorStampedAdapter = rclcpp::TypeAdapter<tobas_kdl_msgs::VectorStamped, tobas_kdl_msgs::msg::VectorStamped>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_kdl_msgs::VectorStamped, tobas_kdl_msgs::msg::VectorStamped);
