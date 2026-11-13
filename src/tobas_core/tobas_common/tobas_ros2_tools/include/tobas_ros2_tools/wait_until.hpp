// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/node.hpp>

namespace tobas::ros2
{
/** Wait for `timeout` until `predicate` returns true. */
bool waitUntil(
  const rclcpp::Node::SharedPtr& _node,
  const std::function<bool()>& _predicate,
  const std::chrono::milliseconds _timeout,
  const std::chrono::milliseconds _interval = std::chrono::milliseconds(10));
}  // namespace tobas::ros2
