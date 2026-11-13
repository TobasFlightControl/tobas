// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/pwm_array.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>

#include "./rosbag_recorder.hpp"

namespace tobas
{
void RosbagRecorderNode::registerCommandSubscribers()
{
  addStandardMsgSub<tobas_msgs::msg::RotorThrustArray>(topic::kRotorThrustsCmd);
  addStandardMsgSub<tobas_msgs::msg::RotorSpeedArray>(topic::kRotorSpeedsCmd);
  addStandardMsgSub<tobas_msgs::msg::IcePropulsionSystemCommand>(topic::kIcePropulsionSystemCmd);
  addStandardMsgSub<tobas_msgs::msg::PwmArray>(topic::kPwmCmd);
  addStandardMsgSub<tobas_msgs::msg::JointCommandArray>(topic::kJointPosCmd);
  addStandardMsgSub<tobas_msgs::msg::JointCommandArray>(topic::kJointVelCmd);
  addStandardMsgSub<tobas_msgs::msg::JointCommandArray>(topic::kJointEffCmd);
}
}  // namespace tobas
