#include "tobas_tools/coordinates.hpp"

#include <tobas_kdl/conversion/coordinates.hpp>

namespace tobas
{
void odometryFrdToFlu(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& dst)
{
  kdl::frameFrdToFlu(src.frame, dst.frame);
  kdl::twistFrdToFlu(src.twist, dst.twist);
  kdl::accelFrdToFlu(src.accel, dst.accel);
}

void odometryFluToFrd(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& dst)
{
  odometryFrdToFlu(src, dst);
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
  tobas_command_msgs::msg::SpeedRollDeltaPitch& dst)
{
  dst.header = src.header;
  dst.speed = src.speed;
  dst.roll = src.roll;
  dst.delta_pitch = -src.delta_pitch;
}

void speedRollDeltaPitchFluToFrd(
  const tobas_command_msgs::msg::SpeedRollDeltaPitch src,
  tobas_command_msgs::msg::SpeedRollDeltaPitch& dst)
{
  speedRollDeltaPitchFrdToFlu(src, dst);
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
