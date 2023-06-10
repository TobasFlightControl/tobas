#pragma once

#include <tobas_msgs/Pose.h>
#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/SpeedRollDeltaPitch.h>

namespace tf
{
void poseNedToNwu(const tobas_msgs::Pose& src, tobas_msgs::Pose& des);
void poseNwuToNed(const tobas_msgs::Pose& src, tobas_msgs::Pose& des);
void poseNedToNwu(tobas_msgs::Pose& arg);
void poseNwuToNed(tobas_msgs::Pose& arg);

void baseStateNedToNwu(const tobas_msgs::BaseState& src, tobas_msgs::BaseState& des);
void baseStateNwuToNed(const tobas_msgs::BaseState& src, tobas_msgs::BaseState& des);
void baseStateNedToNwu(tobas_msgs::BaseState& arg);
void baseStateNwuToNed(tobas_msgs::BaseState& arg);

void speedRollDeltaPitchNedToNwu(
  const tobas_msgs::SpeedRollDeltaPitch src,
  tobas_msgs::SpeedRollDeltaPitch& des);
void speedRollDeltaPitchNwuToNed(
  const tobas_msgs::SpeedRollDeltaPitch src,
  tobas_msgs::SpeedRollDeltaPitch& des);
void speedRollDeltaPitchNedToNwu(tobas_msgs::SpeedRollDeltaPitch& arg);
void speedRollDeltaPitchNwuToNed(tobas_msgs::SpeedRollDeltaPitch& arg);
}  // namespace tf
