// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/tf_listener.hpp"

namespace tobas
{
namespace ros2
{
TransformListener::TransformListener(rclcpp::Node::SharedPtr node)
  : tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_, node)
{
}

std::expected<geometry_msgs::msg::TransformStamped, std::string>
TransformListener::lookupTransform(const std::string& parent, const std::string& child, const rclcpp::Time& time)
{
  try {
    return tf_buffer_.lookupTransform(parent, child, time);
  }
  catch (const tf2::TransformException& e) {
    return std::unexpected(std::string(e.what()));
  }
}
}  // namespace ros2
}  // namespace tobas
