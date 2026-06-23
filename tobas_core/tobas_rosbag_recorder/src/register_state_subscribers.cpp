// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <std_msgs/msg/string.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/latency.hpp>
#include <tobas_msgs/msg/message.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>
#include <tobas_msgs_adapter/gnss.hpp>
#include <tobas_msgs_adapter/imu.hpp>
#include <tobas_msgs_adapter/magnetic_field.hpp>
#include <tobas_msgs_adapter/odometry_stamped.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>
#include <tobas_msgs_adapter/vibration_level.hpp>

#include "./rosbag_recorder.hpp"

namespace tobas
{
void RosbagRecorderNode::registerStateSubscribers()
{
  addStandardMsgSub<tobas_msgs::msg::Message>(topic::kMessage);
  addTypeAdaptedMsgSub<Drone>(drone_, topic::kDrone, true, true);
  addTypeAdaptedMsgSub<kdl::Tree>(tree_, topic::kKdlTree, true, true);
  addStandardMsgSub<std_msgs::msg::String>(topic::kRobotDescription, true, true);
  addStandardMsgSub<tobas_msgs::msg::Battery>(topic::kBattery);
  addStandardMsgSub<tobas_msgs::msg::Cpu>(topic::kCpu);
  addTypeAdaptedMsgSub<tobas_msgs::RCInput>(rcin_, topic::kRcInput);
  addTypeAdaptedMsgSub<tobas_msgs::Imu>(imu_, topic::kImuRaw);
  addTypeAdaptedMsgSub<tobas_msgs::Imu>(imu_, topic::kImuFilt);
  addTypeAdaptedMsgSub<tobas_msgs::MagneticField>(mag_, topic::kMagneticField);
  addStandardMsgSub<tobas_msgs::msg::FluidPressure>(topic::kAirPressure);
  addTypeAdaptedMsgSub<tobas_msgs::Gnss>(gnss_, topic::kGnss);
  addStandardMsgSub<tobas_msgs::msg::RotorStateArray>(topic::kRotorStates);
  addStandardMsgSub<tobas_msgs::msg::RotorLivelinessArray>(topic::kRotorLiv);
  addStandardMsgSub<tobas_msgs::msg::JointStateArray>(topic::kJointStates);
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
