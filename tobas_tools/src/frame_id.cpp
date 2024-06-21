#include "../include/tobas_tools/conversions/frame_id.hpp"

using namespace kdl;
using namespace tobas_msgs;

namespace tobas
{
Rotation rotWorldToFootprint(const Rotation& R_W_B)
{
  return Rotation::RotZ(R_W_B.getYaw());
}

Rotation rotFootprintToLocal(const Rotation& R_W_B)
{
  double roll, pitch, yaw;
  R_W_B.getRPY(roll, pitch, yaw);
  return Rotation::RPY(roll, pitch, 0);
}
}  // namespace tobas
