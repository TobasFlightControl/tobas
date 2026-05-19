// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/node.hpp>

namespace tobas
{
namespace ros2
{
/* `predicate`が`true`を返すまで`timeout`だけ待機する． */
bool waitUntil(
  const rclcpp::Node::SharedPtr& node,
  const std::function<bool()>& predicate,
  const std::chrono::milliseconds timeout,
  const std::chrono::milliseconds interval = std::chrono::milliseconds(10));
}  // namespace ros2
}  // namespace tobas
