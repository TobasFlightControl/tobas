// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/node.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace tobas::ros2
{
class TransformListener
{
public:
  explicit TransformListener(rclcpp::Node::SharedPtr _node);

  std::expected<geometry_msgs::msg::TransformStamped, std::string>
  lookupTransform(const std::string& _parent, const std::string& _child, const rclcpp::Time& _time = rclcpp::Time(0));

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace tobas::ros2
