#pragma once

#include <tobas_kdl/rotation.hpp>
#include <tobas_msgs/PosVelAccYaw.h>

namespace tobas
{
KDL::Rotation rotWorldToFootprint(const KDL::Rotation& R_W_B);

KDL::Rotation rotFootprintToLocal(const KDL::Rotation& R_W_B);

bool changeFrame(
  const uint8_t& frame_id,
  const KDL::Rotation& R_W_B,
  tobas_msgs::PosVelAccYaw& msg);
}  // namespace tobas
