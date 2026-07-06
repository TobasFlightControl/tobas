// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_trajectory_generation/offline/linear.hpp"

#include <algorithm>
#include <stdexcept>

namespace tobas
{
namespace traj
{
LinearSpline::LinearSpline(double p0, double pf, double T) : p0_(p0), T_(T)
{
  if (T > 0.0) {
    v_ = (pf - p0) / T;
  }
  else if (p0 == pf && T == 0.0) {
    v_ = 0.0;
  }
  else {
    throw std::runtime_error("There is no linear spline that satisfies the conditions.");
  }
}

TrajectoryPoint LinearSpline::get(double _t) const noexcept
{
  const auto t = std::clamp(_t, 0.0, T_);
  return { p0_ + v_ * t, v_, 0.0 };
}

double LinearSpline::duration() const noexcept
{
  return T_;
}
}  // namespace traj
}  // namespace tobas
