// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <gz/math/MassMatrix3.hh>

namespace tobas::gazebo
{
gz::math::MassMatrix3d boxInertia(double _sx, double _sy, double _sz, double _mass);
}  // namespace tobas::gazebo
