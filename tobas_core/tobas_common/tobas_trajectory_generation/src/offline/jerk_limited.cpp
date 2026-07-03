// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_trajectory_generation/offline/jerk_limited.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>

#include <tobas_math/core.hpp>

#define EPS 1e-6  // If this is too small, convergence may never occur.

namespace tobas
{
namespace traj
{
JerkLimitedTrajectory::JerkLimitedTrajectory(double p0, double pf, double max_jerk, double max_acc, double max_vel)
  : p0_(p0), pd_(std::abs(pf - p0)), sign_(math::sign(pf - p0)), jm_(max_jerk), am_(max_acc), vm_(max_vel)
{
  assert(jm_ > 0.);
  assert(am_ > 0.);
  assert(vm_ > 0.);

  // Handle the exceptional case where the start and target positions match.
  if (pd_ < EPS) {
    t1_ = t2_ = t3_ = t4_ = 0.;
    return;
  }

  // Adjust the maximum acceleration and velocity to satisfy the time ordering constraints.
  bool ok = false;
  size_t iter = 0;
  while (!ok) {
    ok = true;
    ++iter;

    // Condition for reaching the maximum acceleration.
    if (math::sqr(am_) > vm_ * jm_) {
      am_ = std::sqrt(vm_ * jm_) - EPS;
      ok = false;
    }

    // Condition for reaching the maximum velocity.
    const auto a = 1 / am_;
    const auto b = am_ / jm_;
    const auto c = -pd_;
    if (a * math::sqr(vm_) + b * vm_ + c > 0) {
      vm_ = (std::sqrt(math::sqr(b) - 4 * a * c) - b) / (2 * a) - EPS;
      ok = false;
    }

#ifndef NDEBUG
    std::cout << "Iter " << iter << ": Accel [m/s^2]" << am_ << ", Velocity [m/s]" << vm_ << std::endl;
#endif
  }

  t1_ = am_ / jm_;
  t2_ = vm_ / am_;
  t3_ = t1_ + t2_;
  t4_ = (t3_ + pd_ / vm_) / 2;

  assert(t1_ < t2_);
  assert(t2_ < t3_);
  assert(t3_ < t4_);
}

TrajectoryPoint JerkLimitedTrajectory::get(double t) const noexcept
{
  if (pd_ < EPS) {
    return { p0_, 0., 0. };
  }

  // Restore the origin and movement direction removed at the beginning.
  return { p0_ + sign_ * p(t), sign_ * v(t), sign_ * a(t) };
}

double JerkLimitedTrajectory::duration() const noexcept
{
  return 2 * t4_;
}

double JerkLimitedTrajectory::p(double t) const noexcept
{
  if (t <= 0) {
    return 0;
  }
  else if (t <= t1_) {
    return (jm_ / 6) * math::cube(t);
  }
  else if (t <= t2_) {
    return p(t1_) + (am_ / 2) * t * (t - t1_);
  }
  else if (t <= t3_) {
    return p(t2_) + vm_ * (t - t2_) + (jm_ / 6) * (math::cube(t3_ - t) - math::cube(t1_));
  }
  else if (t <= t4_) {
    return p(t3_) + vm_ * (t - t3_);
  }
  else {
    return pd_ - p(2 * t4_ - t);  // Use point symmetry around t = t4, p = pd/2.
  }
}

double JerkLimitedTrajectory::v(double t) const noexcept
{
  if (t <= 0) {
    return 0;
  }
  else if (t <= t1_) {
    return (jm_ / 2) * math::sqr(t);
  }
  else if (t <= t2_) {
    return v(t1_) + am_ * (t - t1_);
  }
  else if (t <= t3_) {
    return vm_ - v(t3_ - t);  // Use point symmetry around t = t3/2, v = vm/2.
  }
  else if (t <= t4_) {
    return vm_;
  }
  else {
    return v(2 * t4_ - t);  // Use line symmetry around t = t4.
  }
}

double JerkLimitedTrajectory::a(double t) const noexcept
{
  if (t <= 0) {
    return 0;
  }
  else if (t <= t1_) {
    return jm_ * t;
  }
  else if (t <= t2_) {
    return am_;
  }
  else if (t <= t3_) {
    return a(t3_ - t);  // Use line symmetry around t = t3/2.
  }
  else if (t <= t4_) {
    return 0;
  }
  else {
    return -a(2 * t4_ - t);  // Use point symmetry around t = t4, a = 0.
  }
}
}  // namespace traj
}  // namespace tobas
