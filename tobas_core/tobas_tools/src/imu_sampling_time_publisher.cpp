// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_tools/imu_sampling_time_publisher.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/qos.hpp>

namespace tobas
{
ImuSamplingTimePublisher::ImuSamplingTimePublisher(rclcpp::Node* _node)
{
  last_time_ = _node->now();
  pub_ = _node->create_publisher<tobas_msgs::msg::Latency>(topic::kImuSamplingTime, ros2::qos::DefaultQoS());
}

void ImuSamplingTimePublisher::publish(const rclcpp::Time& _cur_time)
{
  auto msg = std::make_unique<tobas_msgs::msg::Latency>();
  msg->header.stamp = _cur_time;
  msg->data = _cur_time - last_time_;
  pub_->publish(std::move(msg));

  last_time_ = _cur_time;
}
}  // namespace tobas
