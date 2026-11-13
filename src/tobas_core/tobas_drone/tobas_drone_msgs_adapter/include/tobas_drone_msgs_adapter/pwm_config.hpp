// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/pwm.hpp>

#include <tobas_drone_msgs/msg/pwm_config.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::PwmConfig, tobas_drone_msgs::msg::PwmConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::PwmConfig;
  using ros_message_type = tobas_drone_msgs::msg::PwmConfig;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.channel = _src.channel;
    _dst.name = _src.name;
    _dst.min_period = _src.period_range.first;
    _dst.max_period = _src.period_range.second;
    _dst.min_value = _src.value_range.first;
    _dst.max_value = _src.value_range.second;
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.channel = _src.channel;
    _dst.name = _src.name;
    _dst.period_range.first = _src.min_period;
    _dst.period_range.second = _src.max_period;
    _dst.value_range.first = _src.min_value;
    _dst.value_range.second = _src.max_value;
  }
};

namespace tobas_drone_msgs
{
using PwmConfigAdapter = rclcpp::TypeAdapter<tobas::PwmConfig, tobas_drone_msgs::msg::PwmConfig>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::PwmConfig, tobas_drone_msgs::msg::PwmConfig);
