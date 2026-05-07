// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace gazebo
{
std::string makeBoxSdf(
  const std::string& name,
  double sx,
  double sy,
  double sz,
  double mass,
  double px,
  double py,
  double pz,
  double rr = 0.,
  double rp = 0.,
  double ry = 0.);
}  // namespace gazebo
}  // namespace tobas
