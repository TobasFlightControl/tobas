#pragma once

#include <tobas_command_msgs/msg/speed_roll_delta_pitch.hpp>
#include <tobas_msgs_adapter/odometry.hpp>

namespace tobas
{
void odometryNedToNwu(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& des);
void odometryNwuToNed(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& des);
void odometryNedToNwu(tobas_msgs::Odometry& arg);
void odometryNwuToNed(tobas_msgs::Odometry& arg);

void speedRollDeltaPitchNedToNwu(
  const tobas_command_msgs::msg::SpeedRollDeltaPitch src,
  tobas_command_msgs::msg::SpeedRollDeltaPitch& des);
void speedRollDeltaPitchNwuToNed(
  const tobas_command_msgs::msg::SpeedRollDeltaPitch src,
  tobas_command_msgs::msg::SpeedRollDeltaPitch& des);
void speedRollDeltaPitchNedToNwu(tobas_command_msgs::msg::SpeedRollDeltaPitch& arg);
void speedRollDeltaPitchNwuToNed(tobas_command_msgs::msg::SpeedRollDeltaPitch& arg);
}  // namespace tobas
