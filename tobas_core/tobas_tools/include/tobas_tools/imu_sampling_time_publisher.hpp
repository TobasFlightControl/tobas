// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/node.hpp>

#include <tobas_msgs/msg/latency.hpp>

namespace tobas
{
class ImuSamplingTimePublisher
{
public:
  explicit ImuSamplingTimePublisher(rclcpp::Node* node);

  void publish(const rclcpp::Time& cur_time);

private:
  rclcpp::Time last_time_;
  rclcpp::Publisher<tobas_msgs::msg::Latency>::SharedPtr pub_;
};
}  // namespace tobas
