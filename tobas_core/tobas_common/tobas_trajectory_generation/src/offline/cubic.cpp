// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_trajectory_generation/offline/cubic.hpp"

#include <algorithm>
#include <stdexcept>

#include <tobas_math/core.hpp>

namespace tobas
{
namespace traj
{
CubicSpline::CubicSpline(double _p0, double _pf, double _t) : T_(_t)
{
  a0_ = _p0;
  a1_ = 0.0;

  if (_t > 0.0) {
    a2_ = (3 / math::sqr(_t)) * (_pf - _p0);
    a3_ = (-2 / math::cube(_t)) * (_pf - _p0);
  }
  else if (_p0 == _pf && _t == 0.0) {
    a2_ = 0.0;
    a3_ = 0.0;
  }
  else {
    throw std::runtime_error("There is no cubic spline that satisfies the conditions.");
  }
}

TrajectoryPoint CubicSpline::get(double _t) const noexcept
{
  const auto t = std::clamp(_t, 0.0, T_);
  const auto p = a0_ + a1_ * t + a2_ * math::sqr(t) + a3_ * math::cube(t);
  const auto v = a1_ + 2 * a2_ * t + 3 * a3_ * math::sqr(t);
  const auto a = 2 * a2_ + 6 * a3_ * t;
  return { p, v, a };
}

double CubicSpline::duration() const noexcept
{
  return T_;
}
}  // namespace traj
}  // namespace tobas
