// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>
#include <tobas_msgs/msg/repulsive_acceleration.hpp>

namespace tobas_msgs
{
struct RepulsiveAcceleration
{
  std_msgs::msg::Header header;
  tobas::kdl::Vector accel;

  using SharedPtr = std::shared_ptr<RepulsiveAcceleration>;
  using ConstSharedPtr = std::shared_ptr<const RepulsiveAcceleration>;
  using UniquePtr = std::unique_ptr<RepulsiveAcceleration>;
  using ConstUniquePtr = std::unique_ptr<const RepulsiveAcceleration>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::RepulsiveAcceleration, tobas_msgs::msg::RepulsiveAcceleration>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::RepulsiveAcceleration;
  using ros_message_type = tobas_msgs::msg::RepulsiveAcceleration;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.accel, dst.accel);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.accel, dst.accel);
  }
};

namespace tobas_msgs
{
using RepulsiveAccelerationAdapter =
  rclcpp::TypeAdapter<tobas_msgs::RepulsiveAcceleration, tobas_msgs::msg::RepulsiveAcceleration>;
}  // namespace tobas_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::RepulsiveAcceleration, tobas_msgs::msg::RepulsiveAcceleration);
