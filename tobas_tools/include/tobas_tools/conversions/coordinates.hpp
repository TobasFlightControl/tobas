#pragma once

#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/SpeedRollDeltaPitch.h>

namespace tf
{
void odometryNedToNwu(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& des);
void odometryNwuToNed(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& des);
void odometryNedToNwu(tobas_msgs::Odometry& arg);
void odometryNwuToNed(tobas_msgs::Odometry& arg);

void speedRollDeltaPitchNedToNwu(
  const tobas_msgs::SpeedRollDeltaPitch src,
  tobas_msgs::SpeedRollDeltaPitch& des);
void speedRollDeltaPitchNwuToNed(
  const tobas_msgs::SpeedRollDeltaPitch src,
  tobas_msgs::SpeedRollDeltaPitch& des);
void speedRollDeltaPitchNedToNwu(tobas_msgs::SpeedRollDeltaPitch& arg);
void speedRollDeltaPitchNwuToNed(tobas_msgs::SpeedRollDeltaPitch& arg);
}  // namespace tf
