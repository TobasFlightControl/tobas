#include <dh_kdl/conversion/coordinates.hpp>

#include "../../../include/tobas_tools/conversions/coordinates.hpp"

namespace tf
{
void poseNedToNwu(const tobas_msgs::Pose& src, tobas_msgs::Pose& des)
{
  vectorNedToNwu(src.pos, des.pos);
  eulerNedToNwu(src.euler, des.euler);
}

void poseNwuToNed(const tobas_msgs::Pose& src, tobas_msgs::Pose& des)
{
  poseNedToNwu(src, des);
}

void poseNedToNwu(tobas_msgs::Pose& arg)
{
  poseNedToNwu(arg, arg);
}

void poseNwuToNed(tobas_msgs::Pose& arg)
{
  poseNwuToNed(arg, arg);
}

void baseStateNedToNwu(const tobas_msgs::BaseState& src, tobas_msgs::BaseState& des)
{
  poseNedToNwu(src.pose, des.pose);
  twistNedToNwu(src.twist, des.twist);
}

void baseStateNwuToNed(const tobas_msgs::BaseState& src, tobas_msgs::BaseState& des)
{
  baseStateNedToNwu(src, des);
}

void baseStateNedToNwu(tobas_msgs::BaseState& arg)
{
  baseStateNedToNwu(arg, arg);
}

void baseStateNwuToNed(tobas_msgs::BaseState& arg)
{
  baseStateNwuToNed(arg, arg);
}

void speedRollDeltaPitchNedToNwu(
  const tobas_msgs::SpeedRollDeltaPitch src,
  tobas_msgs::SpeedRollDeltaPitch& des)
{
  des.speed = src.speed;
  des.roll = src.roll;
  des.delta_pitch = -src.delta_pitch;
}

void speedRollDeltaPitchNwuToNed(
  const tobas_msgs::SpeedRollDeltaPitch src,
  tobas_msgs::SpeedRollDeltaPitch& des)
{
  speedRollDeltaPitchNedToNwu(src, des);
}

void speedRollDeltaPitchNedToNwu(tobas_msgs::SpeedRollDeltaPitch& arg)
{
  speedRollDeltaPitchNedToNwu(arg, arg);
}

void speedRollDeltaPitchNwuToNed(tobas_msgs::SpeedRollDeltaPitch& arg)
{
  speedRollDeltaPitchNwuToNed(arg, arg);
}
}  // namespace tf
