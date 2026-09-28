// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/node.hpp>

namespace tobas
{
namespace ros2
{
class TransformListener
{
public:
  explicit TransformListener(rclcpp::Node::SharedPtr node);

  std::expected<geometry_msgs::msg::TransformStamped, std::string>
  lookupTransform(const std::string& parent, const std::string& child, const rclcpp::Time& time = rclcpp::Time(0));

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace ros2
}  // namespace tobas
