// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace gazebo
{
std::string makeBoxSdf(
  const std::string& _name,
  double _sx,
  double _sy,
  double _sz,
  double _mass,
  double _px = 0.0,
  double _py = 0.0,
  double _pz = 0.0,
  double _rr = 0.0,
  double _rp = 0.0,
  double _ry = 0.0);
}  // namespace gazebo
}  // namespace tobas
