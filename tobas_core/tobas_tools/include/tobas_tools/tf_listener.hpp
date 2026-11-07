// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <rclcpp/node.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <tobas_kdl/frame.hpp>

namespace tobas
{
class TransformListener
{
public:
  template <typename NodeT>
  explicit TransformListener(NodeT&& _node);

  std::expected<kdl::Frame, std::string> lookupTransform(
    const std::string& _parent,
    const std::string& _child,
    const rclcpp::Time& _time = rclcpp::Time(0)) noexcept;

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  kdl::Frame frame_;
};

template <typename NodeT>
TransformListener::TransformListener(NodeT&& _node)
  : tf_buffer_(_node->get_clock())
  , tf_listener_(tf_buffer_, tf2_ros::TransformListener::RequiredInterfaces(*_node), false)
{
  // Process TF subscriptions with the node's executor.
}
}  // namespace tobas
