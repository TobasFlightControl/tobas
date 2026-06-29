// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs_adapter/gnss.hpp>
#include <tobas_msgs_adapter/imu.hpp>
#include <tobas_msgs_adapter/magnetic_field.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>

#include "./rosbag_recorder.hpp"

namespace tobas
{
void RosbagRecorderNode::registerSensorSubscribers()
{
  addStandardMsgSub<tobas_msgs::msg::Battery>(topic::kBattery);
  addStandardMsgSub<tobas_msgs::msg::Cpu>(topic::kCpu);
  addTypeAdaptedMsgSub<tobas_msgs::RCInput>(rcin_, topic::kRcInput);
  addTypeAdaptedMsgSub<tobas_msgs::Imu>(imu_, topic::kImuRaw);
  addTypeAdaptedMsgSub<tobas_msgs::Imu>(imu_, topic::kImuFilt);
  addTypeAdaptedMsgSub<tobas_msgs::MagneticField>(mag_, topic::kMagneticField);
  addStandardMsgSub<tobas_msgs::msg::FluidPressure>(topic::kAirPressure);
  addTypeAdaptedMsgSub<tobas_msgs::Gnss>(gnss_, topic::kGnss);
  addStandardMsgSub<tobas_msgs::msg::RotorStateArray>(topic::kRotorStates);
  addStandardMsgSub<tobas_msgs::msg::JointStateArray>(topic::kJointStates);
}
}  // namespace tobas
