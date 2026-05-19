// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/wait_until.hpp"

namespace ch = std::chrono;

namespace tobas
{
namespace ros2
{
bool waitUntil(
  const rclcpp::Node::SharedPtr& node,
  const std::function<bool()>& predicate,
  const ch::milliseconds timeout,
  const ch::milliseconds interval)
{
  const auto start_time = node->now();
  rclcpp::Rate rate(interval);
  while (rclcpp::ok() && node->now() - start_time < timeout) {
    if (predicate()) {
      return true;
    }
    rate.sleep();
  }
  return false;
}
}  // namespace ros2
}  // namespace tobas
