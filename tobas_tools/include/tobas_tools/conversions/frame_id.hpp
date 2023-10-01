#pragma once

#include <dh_kdl/euler.hpp>

#include <tobas_msgs/PosVelAccYaw.h>

namespace tobas
{
bool changeFrame(
  const uint8_t& frame_id,
  const KDL::Euler& R_gl,
  tobas_msgs::PosVelAccYaw& msg);
}
