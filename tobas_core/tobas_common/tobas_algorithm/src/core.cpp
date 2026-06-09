// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_algorithm/core.hpp"

#include <cassert>
#include <cmath>

#include <tobas_math/core.hpp>
#include <tobas_math/definitions.hpp>

namespace tobas
{
namespace algo
{
double wrapPi(double angle)
{
  angle = std::fmod(angle + M_PI, M_2PI);  // angle を [0, 2π) の範囲に変換
  if (angle < 0.) {
    angle += M_2PI;  // angle が負の場合、範囲を補正
  }
  return angle - M_PI;  // [0, 2π) から [-π, π) へ変換
}

void clamp2d(double& x, double& y, const double& max_length)
{
  assert(max_length >= 0.);

  const auto length = std::sqrt(math::sqr(x) + math::sqr(y));
  if (length > max_length) {
    const auto scale = max_length / length;
    x *= scale;
    y *= scale;
  }
}
}  // namespace algo
}  // namespace tobas
