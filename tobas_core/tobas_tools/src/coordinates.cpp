// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_tools/coordinates.hpp"

#include <tobas_kdl/conversion/coordinates.hpp>

namespace tobas
{
void odometryFrdToFlu(const tobas_msgs::Odometry& _src, tobas_msgs::Odometry& _dst)
{
  kdl::frameFrdToFlu(_src.frame, _dst.frame);
  kdl::twistFrdToFlu(_src.twist, _dst.twist);
  kdl::accelFrdToFlu(_src.accel, _dst.accel);
}

void odometryFluToFrd(const tobas_msgs::Odometry& _src, tobas_msgs::Odometry& _dst)
{
  odometryFrdToFlu(_src, _dst);
}

void odometryFrdToFlu(tobas_msgs::Odometry& _arg)
{
  odometryFrdToFlu(_arg, _arg);
}

void odometryFluToFrd(tobas_msgs::Odometry& _arg)
{
  odometryFluToFrd(_arg, _arg);
}

void speedRollDeltaPitchFrdToFlu(
  const tobas_command_msgs::msg::SpeedRollDeltaPitch _src,
  tobas_command_msgs::msg::SpeedRollDeltaPitch& _dst)
{
  _dst.header = _src.header;
  _dst.speed = _src.speed;
  _dst.roll = _src.roll;
  _dst.delta_pitch = -_src.delta_pitch;
}

void speedRollDeltaPitchFluToFrd(
  const tobas_command_msgs::msg::SpeedRollDeltaPitch _src,
  tobas_command_msgs::msg::SpeedRollDeltaPitch& _dst)
{
  speedRollDeltaPitchFrdToFlu(_src, _dst);
}

void speedRollDeltaPitchFrdToFlu(tobas_command_msgs::msg::SpeedRollDeltaPitch& _arg)
{
  speedRollDeltaPitchFrdToFlu(_arg, _arg);
}

void speedRollDeltaPitchFluToFrd(tobas_command_msgs::msg::SpeedRollDeltaPitch& _arg)
{
  speedRollDeltaPitchFluToFrd(_arg, _arg);
}
}  // namespace tobas
