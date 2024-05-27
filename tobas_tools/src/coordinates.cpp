#include <tobas_kdl/conversion/coordinates.hpp>

#include "../include/tobas_tools/conversions/coordinates.hpp"

namespace tf
{
void odometryNedToNwu(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& des)
{
  des.header = src.header;
  des.status = src.status;
  frameNedToNwu(src.frame, des.frame);
  twistNedToNwu(src.twist, des.twist);
}

void odometryNwuToNed(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& des)
{
  odometryNedToNwu(src, des);
}

void odometryNedToNwu(tobas_msgs::Odometry& arg)
{
  odometryNedToNwu(arg, arg);
}

void odometryNwuToNed(tobas_msgs::Odometry& arg)
{
  odometryNwuToNed(arg, arg);
}

void speedRollDeltaPitchNedToNwu(const tobas_msgs::SpeedRollDeltaPitch src, tobas_msgs::SpeedRollDeltaPitch& des)
{
  des.speed = src.speed;
  des.roll = src.roll;
  des.delta_pitch = -src.delta_pitch;
}

void speedRollDeltaPitchNwuToNed(const tobas_msgs::SpeedRollDeltaPitch src, tobas_msgs::SpeedRollDeltaPitch& des)
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
