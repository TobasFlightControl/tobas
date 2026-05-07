// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace tobas
{
namespace ros2
{
bool isPresent(const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr& graph, const std::string& target_fqn);

bool waitUntilNodeGone(
  const rclcpp::Node::SharedPtr& node,
  const std::string& target_fqn,  // FQN = Fully Qualified Name
  std::chrono::nanoseconds timeout = std::chrono::nanoseconds::max());
}  // namespace ros2
}  // namespace tobas
