#pragma once

#include <tobas_command_msgs/msg/speed_roll_delta_pitch.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

namespace tobas
{
void odometryFrdToFlu(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& des);
void odometryFluToFrd(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& des);
void odometryFrdToFlu(tobas_msgs::Odometry& arg);
void odometryFluToFrd(tobas_msgs::Odometry& arg);

void speedRollDeltaPitchFrdToFlu(
  const tobas_command_msgs::msg::SpeedRollDeltaPitch src,
  tobas_command_msgs::msg::SpeedRollDeltaPitch& des);
void speedRollDeltaPitchFluToFrd(
  const tobas_command_msgs::msg::SpeedRollDeltaPitch src,
  tobas_command_msgs::msg::SpeedRollDeltaPitch& des);
void speedRollDeltaPitchFrdToFlu(tobas_command_msgs::msg::SpeedRollDeltaPitch& arg);
void speedRollDeltaPitchFluToFrd(tobas_command_msgs::msg::SpeedRollDeltaPitch& arg);
}  // namespace tobas
