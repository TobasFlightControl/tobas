// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_command_msgs/msg/speed_roll_delta_pitch.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

namespace tobas
{
void odometryFrdToFlu(const tobas_msgs::Odometry& _src, tobas_msgs::Odometry& _des);
void odometryFluToFrd(const tobas_msgs::Odometry& _src, tobas_msgs::Odometry& _des);
void odometryFrdToFlu(tobas_msgs::Odometry& _arg);
void odometryFluToFrd(tobas_msgs::Odometry& _arg);

void speedRollDeltaPitchFrdToFlu(
  const tobas_command_msgs::msg::SpeedRollDeltaPitch _src,
  tobas_command_msgs::msg::SpeedRollDeltaPitch& _des);
void speedRollDeltaPitchFluToFrd(
  const tobas_command_msgs::msg::SpeedRollDeltaPitch _src,
  tobas_command_msgs::msg::SpeedRollDeltaPitch& _des);
void speedRollDeltaPitchFrdToFlu(tobas_command_msgs::msg::SpeedRollDeltaPitch& _arg);
void speedRollDeltaPitchFluToFrd(tobas_command_msgs::msg::SpeedRollDeltaPitch& _arg);
}  // namespace tobas
