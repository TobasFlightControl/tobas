// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <string>

#include <rclcpp/node.hpp>

namespace tobas
{
namespace ros2
{
bool waitUntilNodeGone(
  const rclcpp::Node::SharedPtr& node,
  const std::string& target_fqn,  // FQN = Fully Qualified Name
  std::chrono::milliseconds timeout = std::chrono::milliseconds(-1));
}  // namespace ros2
}  // namespace tobas
