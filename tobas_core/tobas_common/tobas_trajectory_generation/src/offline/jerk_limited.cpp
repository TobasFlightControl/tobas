// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_trajectory_generation/offline/jerk_limited.hpp"

#include <cassert>
#include <iostream>

#include <tobas_math/core.hpp>

namespace tobas
{
namespace traj
{
namespace
{
constexpr auto kEps = 1e-6;  // If this is too small, convergence may never occur.
}  // namespace

JerkLimitedTrajectory::JerkLimitedTrajectory(double _p0, double _pf, double _max_jerk, double _max_acc, double _max_vel)
  : p0_(_p0), pd_(std::abs(_pf - _p0)), sign_(math::sign(_pf - _p0)), jm_(_max_jerk), am_(_max_acc), vm_(_max_vel)
{
  assert(jm_ > 0.0);
  assert(am_ > 0.0);
  assert(vm_ > 0.0);

  // Handle the exceptional case where the start and target positions match.
  if (pd_ < kEps) {
    t1_ = t2_ = t3_ = t4_ = 0.0;
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
      am_ = std::sqrt(vm_ * jm_) - kEps;
      ok = false;
    }

    // Condition for reaching the maximum velocity.
    const auto a = 1 / am_;
    const auto b = am_ / jm_;
    const auto c = -pd_;
    if (a * math::sqr(vm_) + b * vm_ + c > 0) {
      vm_ = (std::sqrt(math::sqr(b) - 4 * a * c) - b) / (2 * a) - kEps;
      ok = false;
    }

#ifndef NDEBUG
    std::cout << "Iter " << iter << ": Accel [m/s^2]" << am_ << ", Velocity [m/s]" << vm_ << '\n';
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

TrajectoryPoint JerkLimitedTrajectory::get(double _t) const noexcept
{
  if (pd_ < kEps) {
    return { p0_, 0.0, 0.0 };
  }

  // Restore the origin and movement direction removed at the beginning.
  return { p0_ + sign_ * p(_t), sign_ * v(_t), sign_ * a(_t) };
}

double JerkLimitedTrajectory::duration() const noexcept
{
  return 2 * t4_;
}

double JerkLimitedTrajectory::p(double _t) const noexcept
{
  if (_t <= 0) {
    return 0;
  }
  else if (_t <= t1_) {
    return (jm_ / 6) * math::cube(_t);
  }
  else if (_t <= t2_) {
    return p(t1_) + (am_ / 2) * _t * (_t - t1_);
  }
  else if (_t <= t3_) {
    return p(t2_) + vm_ * (_t - t2_) + (jm_ / 6) * (math::cube(t3_ - _t) - math::cube(t1_));
  }
  else if (_t <= t4_) {
    return p(t3_) + vm_ * (_t - t3_);
  }
  else {
    return pd_ - p(2 * t4_ - _t);  // Use point symmetry around t = t4, p = pd/2.
  }
}

double JerkLimitedTrajectory::v(double _t) const noexcept
{
  if (_t <= 0) {
    return 0;
  }
  else if (_t <= t1_) {
    return (jm_ / 2) * math::sqr(_t);
  }
  else if (_t <= t2_) {
    return v(t1_) + am_ * (_t - t1_);
  }
  else if (_t <= t3_) {
    return vm_ - v(t3_ - _t);  // Use point symmetry around t = t3/2, v = vm/2.
  }
  else if (_t <= t4_) {
    return vm_;
  }
  else {
    return v(2 * t4_ - _t);  // Use line symmetry around t = t4.
  }
}

double JerkLimitedTrajectory::a(double _t) const noexcept
{
  if (_t <= 0) {
    return 0;
  }
  else if (_t <= t1_) {
    return jm_ * _t;
  }
  else if (_t <= t2_) {
    return am_;
  }
  else if (_t <= t3_) {
    return a(t3_ - _t);  // Use line symmetry around t = t3/2.
  }
  else if (_t <= t4_) {
    return 0;
  }
  else {
    return -a(2 * t4_ - _t);  // Use point symmetry around t = t4, a = 0.
  }
}
}  // namespace traj
}  // namespace tobas
