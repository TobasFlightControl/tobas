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
double wrapPi(double _angle)
{
  _angle = std::fmod(_angle + M_PI, M_2PI);  // Convert `angle` to the range [0, 2pi).
  if (_angle < 0.0) {
    _angle += M_2PI;  // Correct the range if `angle` is negative.
  }
  return _angle - M_PI;  // Convert from [0, 2pi) to [-pi, pi).
}

void clamp2d(double& _x, double& _y, double _max_length)
{
  assert(_max_length >= 0.0);

  const auto length = std::hypot(_x, _y);
  if (length > _max_length) {
    const auto scale = _max_length / length;
    _x *= scale;
    _y *= scale;
  }
}
}  // namespace algo
}  // namespace tobas
