// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <gz/math/MassMatrix3.hh>

namespace tobas
{
namespace gazebo
{
gz::math::MassMatrix3d boxInertia(double sx, double sy, double sz, double mass);
}  // namespace gazebo
}  // namespace tobas
