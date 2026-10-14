// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_tools/control_latency_publisher.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/qos.hpp>
#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
ControlLatencyPublisher::ControlLatencyPublisher(rclcpp::Node* _node) : node_(_node)
{
  pub_ = _node->create_publisher<tobas_msgs::msg::Latency>(topic::kControlLatency, ros2::qos::DefaultQoS());
}

void ControlLatencyPublisher::publish(const builtin_interfaces::msg::Time& _start_time)
{
  const builtin_interfaces::msg::Time cur_time(node_->now());  // Drop the clock type to avoid exceptions.

  auto msg = std::make_unique<tobas_msgs::msg::Latency>();
  msg->header.stamp = cur_time;
  msg->data = cur_time - _start_time;

  pub_->publish(std::move(msg));
}
}  // namespace tobas
