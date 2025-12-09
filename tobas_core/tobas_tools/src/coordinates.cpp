#include "tobas_tools/coordinates.hpp"

#include <tobas_kdl/conversion/coordinates.hpp>

namespace tobas
{
void odometryFrdToFlu(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& des)
{
  des.header = src.header;
  des.status = src.status;
  kdl::frameFrdToFlu(src.frame, des.frame);
  kdl::twistFrdToFlu(src.twist, des.twist);
}

void odometryFluToFrd(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& des)
{
  odometryFrdToFlu(src, des);
}

void odometryFrdToFlu(tobas_msgs::Odometry& arg)
{
  odometryFrdToFlu(arg, arg);
}

void odometryFluToFrd(tobas_msgs::Odometry& arg)
{
  odometryFluToFrd(arg, arg);
}

void speedRollDeltaPitchFrdToFlu(
  const tobas_command_msgs::msg::SpeedRollDeltaPitch src,
  tobas_command_msgs::msg::SpeedRollDeltaPitch& des)
{
  des.speed = src.speed;
  des.roll = src.roll;
  des.delta_pitch = -src.delta_pitch;
}

void speedRollDeltaPitchFluToFrd(
  const tobas_command_msgs::msg::SpeedRollDeltaPitch src,
  tobas_command_msgs::msg::SpeedRollDeltaPitch& des)
{
  speedRollDeltaPitchFrdToFlu(src, des);
}

void speedRollDeltaPitchFrdToFlu(tobas_command_msgs::msg::SpeedRollDeltaPitch& arg)
{
  speedRollDeltaPitchFrdToFlu(arg, arg);
}

void speedRollDeltaPitchFluToFrd(tobas_command_msgs::msg::SpeedRollDeltaPitch& arg)
{
  speedRollDeltaPitchFluToFrd(arg, arg);
}
}  // namespace tobas
