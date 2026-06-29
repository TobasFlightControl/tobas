// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/latency.hpp>
#include <tobas_msgs/msg/message.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>
#include <tobas_msgs_adapter/odometry_stamped.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/vibration_level.hpp>

#include "./rosbag_recorder.hpp"

namespace tobas
{
void RosbagRecorderNode::registerStateSubscribers()
{
  addStandardMsgSub<tobas_msgs::msg::Message>(topic::kMessage);
  addStandardMsgSub<tobas_msgs::msg::RotorLivelinessArray>(topic::kRotorLiv);
  addTypeAdaptedMsgSub<tobas_msgs::OdometryWithCovarianceStamped>(odom_cov_, topic::kOdometry);
  addTypeAdaptedMsgSub<tobas_msgs::OdometryStamped>(odom_, topic::kTrajSetpoint);
  addStandardMsgSub<tobas_msgs::msg::Latency>(topic::kImuSamplingTime);
  addStandardMsgSub<tobas_msgs::msg::Latency>(topic::kControlLatency);
  addStandardMsgSub<tobas_msgs::msg::Arming>(topic::kArming);
  addStandardMsgSub<tobas_msgs::msg::VehicleHealth>(topic::kVehicleHealth);
  addTypeAdaptedMsgSub<tobas_msgs::VibrationLevel>(vibe_, topic::kVibrationLevel);
  addTypeAdaptedMsgSub<tobas_kdl_msgs::WrenchStamped>(dist_force_, topic::kDisturbanceForce);
}
}  // namespace tobas
