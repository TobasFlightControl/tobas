// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/tf_listener.hpp"

namespace tobas::ros2
{
TransformListener::TransformListener(rclcpp::Node::SharedPtr _node)
  : tf_buffer_(_node->get_clock()), tf_listener_(tf_buffer_, _node)
{
}

std::expected<geometry_msgs::msg::TransformStamped, std::string>
TransformListener::lookupTransform(const std::string& _parent, const std::string& _child, const rclcpp::Time& _time)
{
  try {
    return tf_buffer_.lookupTransform(_parent, _child, _time);
  }
  catch (const tf2::TransformException& e) {
    return std::unexpected(std::string(e.what()));
  }
}
}  // namespace tobas::ros2
