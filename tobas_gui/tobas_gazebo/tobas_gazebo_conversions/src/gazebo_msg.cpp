// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_conversions/gazebo_msg.hpp"

namespace tobas::gazebo
{
void vector3dGzToMsg(const gz::math::Vector3d& _g, gz::msgs::Vector3d& _m)
{
  _m.set_x(_g.X());
  _m.set_y(_g.Y());
  _m.set_z(_g.Z());
}

void vector3dMsgToGz(const gz::msgs::Vector3d& _m, gz::math::Vector3d& _g)
{
  _g.X(_m.x());
  _g.Y(_m.y());
  _g.Z(_m.z());
}
}  // namespace tobas::gazebo
