// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/propulsion_system/rotor.hpp>

#include <tobas_drone_msgs/msg/rotor_config.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::RotorConfig, tobas_drone_msgs::msg::RotorConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::RotorConfig;
  using ros_message_type = tobas_drone_msgs::msg::RotorConfig;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.link_name = src.link_name;
    dst.direction = static_cast<uint8_t>(src.direction);
    dst.tilt_joint_name = src.tilt_joint_name;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.link_name = src.link_name;
    dst.direction = static_cast<tobas::TurningDirection>(src.direction);
    dst.tilt_joint_name = src.tilt_joint_name;
  }
};

namespace tobas_drone_msgs
{
using RotorConfigAdapter = rclcpp::TypeAdapter<tobas::RotorConfig, tobas_drone_msgs::msg::RotorConfig>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::RotorConfig, tobas_drone_msgs::msg::RotorConfig);
