// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_mission_items/mission.hpp>

#include <tobas_mission_msgs/msg/mission_item.hpp>

namespace tobas_mission_msgs
{
using MissionItemAdapter = rclcpp::TypeAdapter<tobas::mission::MissionItem, tobas_mission_msgs::msg::MissionItem>;
}  // namespace tobas_mission_msgs

template <>
struct rclcpp::TypeAdapter<tobas::mission::MissionItem, tobas_mission_msgs::msg::MissionItem>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::mission::MissionItem;
  using ros_message_type = tobas_mission_msgs::msg::MissionItem;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.type = static_cast<uint8_t>(src.type);
    dst.data = src.data;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.type = static_cast<tobas::mission::Type>(src.type);
    dst.data = src.data;
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::mission::MissionItem, tobas_mission_msgs::msg::MissionItem);
