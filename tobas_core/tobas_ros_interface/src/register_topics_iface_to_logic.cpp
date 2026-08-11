// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_real_common/ros_interface.hpp>

#include <tobas_command_msgs/msg/accel.hpp>
#include <tobas_command_msgs/msg/accel_pitch_yaw.hpp>
#include <tobas_command_msgs/msg/accel_yaw.hpp>
#include <tobas_command_msgs/msg/angle.hpp>
#include <tobas_command_msgs/msg/angle_throttle.hpp>
#include <tobas_command_msgs/msg/pos_vel_acc.hpp>
#include <tobas_command_msgs/msg/pos_vel_acc_pitch_yaw.hpp>
#include <tobas_command_msgs/msg/pos_vel_acc_yaw.hpp>
#include <tobas_command_msgs/msg/rate.hpp>
#include <tobas_command_msgs/msg/rate_throttle.hpp>
#include <tobas_command_msgs/msg/speed_roll_delta_pitch.hpp>
#include <tobas_kdl_msgs/msg/frame_with_covariance_stamped.hpp>
#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/pwm_array.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>

#include "./ros_interface.hpp"

namespace tobas
{
void RosInterfaceNode::registerTopicsIfaceToLogic()
{
  addTopicIfaceToLogic<tobas_kdl_msgs::msg::FrameWithCovarianceStamped>(topic::kExternalPose, topic::kExternalPose);
  addTopicIfaceToLogic<tobas_msgs::msg::RotorSpeedArray>(topic::kRotorSpeedsCmd, topic::kRotorSpeedsCmd);
  addTopicIfaceToLogic<tobas_msgs::msg::IcePropulsionSystemCommand>(
    topic::kIcePropulsionSystemCmd, topic::kIcePropulsionSystemCmd);
  addTopicIfaceToLogic<tobas_msgs::msg::PwmArray>(topic::kPwmCmd, topic::kPwmCmd);
  addTopicIfaceToLogic<tobas_command_msgs::msg::Rate>(topic::kRateCmd, topic::kRateCmd);
  addTopicIfaceToLogic<tobas_command_msgs::msg::RateThrottle>(topic::kRateThrotCmd, topic::kRateThrotCmd);
  addTopicIfaceToLogic<tobas_command_msgs::msg::Angle>(topic::kAngleCmd, topic::kAngleCmd);
  addTopicIfaceToLogic<tobas_command_msgs::msg::AngleThrottle>(topic::kAngleThrotCmd, topic::kAngleThrotCmd);
  addTopicIfaceToLogic<tobas_command_msgs::msg::Accel>(topic::kAccelCmd, topic::kAccelCmd);
  addTopicIfaceToLogic<tobas_command_msgs::msg::AccelYaw>(topic::kAccelYawCmd, topic::kAccelYawCmd);
  addTopicIfaceToLogic<tobas_command_msgs::msg::AccelPitchYaw>(topic::kAccelPitchYawCmd, topic::kAccelPitchYawCmd);
  addTopicIfaceToLogic<tobas_command_msgs::msg::PosVelAcc>(topic::kPosVelAccCmd, topic::kPosVelAccCmd);
  addTopicIfaceToLogic<tobas_command_msgs::msg::PosVelAccYaw>(topic::kPosVelAccYawCmd, topic::kPosVelAccYawCmd);
  addTopicIfaceToLogic<tobas_command_msgs::msg::PosVelAccPitchYaw>(
    topic::kPosVelAccPitchYawCmd, topic::kPosVelAccPitchYawCmd);
  addTopicIfaceToLogic<tobas_command_msgs::msg::SpeedRollDeltaPitch>(
    topic::kSpeedRollDpitchCmd, topic::kSpeedRollDpitchCmd);
  addTopicIfaceToLogic<tobas_msgs::msg::JointCommandArray>(topic::kJointPosCmd, topic::kJointPosCmd);
  addTopicIfaceToLogic<tobas_msgs::msg::JointCommandArray>(topic::kJointVelCmd, topic::kJointVelCmd);
  addTopicIfaceToLogic<tobas_msgs::msg::JointCommandArray>(topic::kJointEffCmd, topic::kJointEffCmd);
}
}  // namespace tobas
