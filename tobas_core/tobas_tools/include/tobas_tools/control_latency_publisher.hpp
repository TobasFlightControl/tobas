// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/node.hpp>

#include <tobas_msgs/msg/latency.hpp>

namespace tobas
{
class ControlLatencyPublisher
{
public:
  explicit ControlLatencyPublisher(rclcpp::Node* node);

  void publish(const builtin_interfaces::msg::Time& start_time);

private:
  rclcpp::Node* const node_;
  rclcpp::Publisher<tobas_msgs::msg::Latency>::SharedPtr pub_;
};
}  // namespace tobas
