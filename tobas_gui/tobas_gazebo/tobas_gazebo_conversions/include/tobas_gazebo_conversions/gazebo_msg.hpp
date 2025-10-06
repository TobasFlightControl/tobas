#pragma once

#include <gz/msgs/vector3d.pb.h>
#include <gz/math/Vector3.hh>

namespace gazebo
{
void vector3dGzToMsg(const gz::math::Vector3d& g, gz::msgs::Vector3d& m);
void vector3dMsgToGz(const gz::msgs::Vector3d& m, gz::math::Vector3d& g);
}  // namespace gazebo
