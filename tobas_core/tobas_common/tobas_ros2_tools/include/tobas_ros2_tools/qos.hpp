#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ros2
{
namespace qos
{
static constexpr bool kDefaultLatch = false;
static constexpr bool kDefaultReliable = false;
static constexpr size_t kDefaultQueueSize = 1;
}  // namespace qos

rclcpp::QoS makeQoS(
  bool latch = qos::kDefaultLatch,
  bool reliable = qos::kDefaultReliable,
  size_t queue_size = qos::kDefaultQueueSize);
}  // namespace ros2
