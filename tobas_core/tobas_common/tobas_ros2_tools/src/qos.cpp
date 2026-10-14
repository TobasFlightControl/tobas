// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/qos.hpp"

namespace tobas::ros2::qos
{
QoS::QoS(bool _latch, bool _reliable, size_t _queue_size) : rclcpp::QoS(rclcpp::KeepLast(_queue_size))
{
  if (_latch) {
    durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
  else {
    durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  if (_reliable) {
    reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  }
  else {
    reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  }
}

DefaultQoS::DefaultQoS() : QoS(kDefaultLatch, kDefaultReliable, kDefaultQueueSize)
{
}
}  // namespace tobas::ros2::qos
