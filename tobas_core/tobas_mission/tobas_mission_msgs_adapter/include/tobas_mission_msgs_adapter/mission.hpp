// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <ranges>

#include <rclcpp/type_adapter.hpp>

#include <tobas_mission_items/mission.hpp>

#include <tobas_mission_msgs/msg/mission.hpp>

#include "./mission_item.hpp"

namespace tobas_mission_msgs
{
using MissionAdapter = rclcpp::TypeAdapter<tobas::mission::Mission, tobas_mission_msgs::msg::Mission>;
}  // namespace tobas_mission_msgs

template <>
struct rclcpp::TypeAdapter<tobas::mission::Mission, tobas_mission_msgs::msg::Mission>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::mission::Mission;
  using ros_message_type = tobas_mission_msgs::msg::Mission;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.items.resize(src.items.size());
    for (const auto& [src_item, dst_item] : std::views::zip(src.items, dst.items)) {
      tobas_mission_msgs::MissionItemAdapter::convert_to_ros_message(src_item, dst_item);
    }
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.items.resize(src.items.size());
    for (const auto& [src_item, dst_item] : std::views::zip(src.items, dst.items)) {
      tobas_mission_msgs::MissionItemAdapter::convert_to_custom(src_item, dst_item);
    }
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::mission::Mission, tobas_mission_msgs::msg::Mission);
