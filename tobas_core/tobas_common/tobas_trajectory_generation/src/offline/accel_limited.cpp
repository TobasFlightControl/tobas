// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_trajectory_generation/offline/accel_limited.hpp"

#include <cassert>

#include <tobas_math/core.hpp>

namespace tobas
{
namespace traj
{
namespace
{
double solveQuadraticEquationPositive(double a, double b, double c)
{
  assert(a > 0);
  const auto d = math::sqr(b) - 4 * a * c;
  assert(d > 0);
  return (-b + std::sqrt(d)) / (2 * a);
}
}  // namespace

AccelLimitedTrajectory::AccelLimitedTrajectory(double p0, double v0, double pf, double vf, double max_acc)
  : p0_(p0), v0_(v0), pf_(pf), vf_(vf), am_(max_acc)
{
  assert(am_ > 0.);

  const auto td = (vf_ - v0_) / am_;
  const auto am_2 = am_ / 2;

  s_ = (pf_ - p0_) + std::abs(vf_ - v0_) * (vf_ + v0_) / (2 * am_);

  if (s_ < 0) {  // -a -> +a
    ts_ = solveQuadraticEquationPositive(am_, -2 * v0_, pf_ - p0_ - vf_ * td + am_2 * math::sqr(td));
    tf_ = 2 * ts_ + td;
  }
  else {  // +a -> -a
    ts_ = solveQuadraticEquationPositive(am_, 2 * v0_, p0_ - pf_ - vf_ * td + am_2 * math::sqr(td));
    tf_ = 2 * ts_ - td;
  }
}

TrajectoryPoint AccelLimitedTrajectory::get(double t) const noexcept
{
  const auto am_2 = am_ / 2;

  if (s_ < 0) {  // -a -> +a
    if (t < ts_) {
      return { p0_ + v0_ * t - am_2 * math::sqr(t), v0_ - am_ * t, -am_ };
    }
    else if (t < tf_) {
      const auto tr = tf_ - t;
      return { pf_ - vf_ * tr + am_2 * math::sqr(tr), vf_ - am_ * tr, am_ };
    }
    else {
      return { pf_, vf_, 0. };
    }
  }
  else {  // +a -> -a
    if (t < ts_) {
      return { p0_ + v0_ * t + am_2 * math::sqr(t), v0_ + am_ * t, am_ };
    }
    else if (t < tf_) {
      const auto tr = tf_ - t;
      return { pf_ - vf_ * tr - am_2 * math::sqr(tr), vf_ + am_ * tr, -am_ };
    }
    else {
      return { pf_, vf_, 0. };
    }
  }
}

double AccelLimitedTrajectory::duration() const noexcept
{
  return tf_;
}
}  // namespace traj
}  // namespace tobas
