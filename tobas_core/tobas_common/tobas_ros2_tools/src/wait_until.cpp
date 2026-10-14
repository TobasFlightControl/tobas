// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/wait_until.hpp"

namespace ch = std::chrono;

namespace tobas::ros2
{
bool waitUntil(
  const rclcpp::Node::SharedPtr& _node,
  const std::function<bool()>& _predicate,
  const ch::milliseconds _timeout,
  const ch::milliseconds _interval)
{
  const auto start_time = _node->now();
  rclcpp::Rate rate(_interval);
  while (rclcpp::ok() && _node->now() - start_time < _timeout) {
    if (_predicate()) {
      return true;
    }
    rate.sleep();
  }
  return false;
}
}  // namespace tobas::ros2
