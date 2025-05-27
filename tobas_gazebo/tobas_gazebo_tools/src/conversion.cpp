#include "tobas_gazebo_tools/conversion.hpp"

namespace gazebo
{
void vector3dGzToMsg(const gz::math::Vector3d& g, gz::msgs::Vector3d& m)
{
  m.set_x(g.X());
  m.set_y(g.Y());
  m.set_z(g.Z());
}

void vector3dMsgToGz(const gz::msgs::Vector3d& m, gz::math::Vector3d& g)
{
  g.X(m.x());
  g.Y(m.y());
  g.Z(m.z());
}
}  // namespace gazebo
