// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_algorithm/core.hpp"

#include <cassert>
#include <cmath>

#include <tobas_math/definitions.hpp>

namespace tobas
{
namespace algo
{
double wrapPi(double angle)
{
  angle = std::fmod(angle + M_PI, M_2PI);  // Convert `angle` to the range [0, 2pi).
  if (angle < 0.0) {
    angle += M_2PI;  // Correct the range if `angle` is negative.
  }
  return angle - M_PI;  // Convert from [0, 2pi) to [-pi, pi).
}

void clamp2d(double& x, double& y, const double& max_length)
{
  assert(max_length >= 0.0);

  const auto length = std::hypot(x, y);
  if (length > max_length) {
    const auto scale = max_length / length;
    x *= scale;
    y *= scale;
  }
}
}  // namespace algo
}  // namespace tobas
