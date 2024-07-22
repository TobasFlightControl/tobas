#include "../include/tobas_tools/conversions/frame_id.hpp"

namespace tobas
{
kdl::Rotation rotWorldToFootprint(const kdl::Rotation& R_W_B)
{
  return kdl::Rotation::RotZ(R_W_B.getYaw());
}

kdl::Rotation rotFootprintToLocal(const kdl::Rotation& R_W_B)
{
  double roll, pitch, yaw;
  R_W_B.getRPY(roll, pitch, yaw);
  return kdl::Rotation::RPY(roll, pitch, 0);
}
}  // namespace tobas
