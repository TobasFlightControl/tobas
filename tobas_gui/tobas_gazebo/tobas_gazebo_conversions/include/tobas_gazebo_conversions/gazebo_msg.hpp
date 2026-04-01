// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <gz/msgs/vector3d.pb.h>
#include <gz/math/Vector3.hh>

namespace tobas
{
namespace gazebo
{
void vector3dGzToMsg(const gz::math::Vector3d& g, gz::msgs::Vector3d& m);
void vector3dMsgToGz(const gz::msgs::Vector3d& m, gz::math::Vector3d& g);
}  // namespace gazebo
}  // namespace tobas
