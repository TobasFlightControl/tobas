// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_system_plugins/inertia.hpp"

#include <cassert>

#include <tobas_math/core.hpp>

namespace tobas
{
namespace gazebo
{
std::tuple<double, double, double> boxInertia(double sx, double sy, double sz, double mass)
{
  assert(sx > 0.0);
  assert(sy > 0.0);
  assert(sz > 0.0);
  assert(mass > 0.0);

  const auto sx2 = math::sqr(sx);
  const auto sy2 = math::sqr(sy);
  const auto sz2 = math::sqr(sz);
  const auto c = mass / 12;
  const double ixx = c * (sy2 + sz2);
  const double iyy = c * (sz2 + sx2);
  const double izz = c * (sx2 + sy2);

  return { ixx, iyy, izz };
}
}  // namespace gazebo
}  // namespace tobas
