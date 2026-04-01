// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/qos.hpp"

namespace tobas
{
namespace ros2
{
namespace qos
{
QoS::QoS(bool latch, bool reliable, size_t queue_size) : rclcpp::QoS(rclcpp::KeepLast(queue_size))
{
  if (latch) {
    durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
  else {
    durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  if (reliable) {
    reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  }
  else {
    reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  }
}

DefaultQoS::DefaultQoS() : QoS(kDefaultLatch, kDefaultReliable, kDefaultQueueSize)
{
}
}  // namespace qos
}  // namespace ros2
}  // namespace tobas
