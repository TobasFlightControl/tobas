// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_mission_execution_mc/stop_trajectory_generator.hpp"

#include <algorithm>
#include <cassert>

#include <tobas_math/core.hpp>

namespace tobas::mission
{
namespace
{
constexpr double kEps = 1e-6;
}  // namespace

StopTrajectory::StopTrajectory(double _p0, double _v0, double _a0, double _am, double _jm)
  : p0_(_p0), v0_(_v0), a0_(_a0), am_(_am), jm_(_jm)
{
  assert(v0_ >= 0.0);
  assert(am_ > 0.0);
  assert(jm_ > 0.0);

  // Special case when already stopped.
  if (v0_ < kEps) {
    t1_ = t2_ = t3_ = 0.0;
    return;
  }

  // Adjust the maximum jerk and acceleration to satisfy time-ordering constraints.
  // The initial state cannot be changed, so the constraints must be relaxed.
  jm_ = std::max(jm_, math::sqr(a0_) / (2 * v0_));
  am_ = std::clamp(am_, -a0_, std::sqrt(math::sqr(a0_) / 2 + v0_ * jm_));

  t1_ = (a0_ + am_) / jm_;
  t2_ = math::sqr(a0_) / (2 * am_ * jm_) + v0_ / am_ + a0_ / jm_;
  t3_ = t2_ + am_ / jm_;

  assert(0.0 <= t1_);
  assert(t1_ <= t2_);
  assert(t2_ <= t3_);
}

traj::TrajectoryPoint StopTrajectory::get(double _t) const noexcept
{
  if (v0_ < kEps) {
    return { p0_, 0.0, 0.0 };
  }

  return { p(_t), v(_t), a(_t) };
}

double StopTrajectory::duration() const noexcept
{
  return t3_;
}

double StopTrajectory::p(double _t) const noexcept
{
  if (_t <= 0) {
    return p0_;
  }
  else if (_t <= t1_) {
    return p0_ + v0_ * _t + (a0_ / 2) * math::sqr(_t) - (jm_ / 6) * math::cube(_t);
  }
  else if (_t <= t2_) {
    const auto dt = _t - t1_;
    return p(t1_) + v(t1_) * dt - (am_ / 2) * math::sqr(dt);
  }
  else if (_t <= t3_) {
    const auto dt = _t - t2_;
    return p(t2_) + v(t2_) * dt + (a(t2_) / 2) * math::sqr(dt) + (jm_ / 6) * math::cube(dt);
  }
  else {
    return p(t3_);
  }
}

double StopTrajectory::v(double _t) const noexcept
{
  if (_t <= 0) {
    return v0_;
  }
  else if (_t <= t1_) {
    return v0_ + a0_ * _t - (jm_ / 2) * math::sqr(_t);
  }
  else if (_t <= t2_) {
    return v(t1_) - am_ * (_t - t1_);
  }
  else if (_t <= t3_) {
    const auto dt = _t - t2_;
    return v(t2_) + a(t2_) * dt + (jm_ / 2) * math::sqr(dt);
  }
  else {
    return 0;
  }
}

double StopTrajectory::a(double _t) const noexcept
{
  if (_t <= 0) {
    return a0_;
  }
  else if (_t <= t1_) {
    return a0_ - jm_ * _t;
  }
  else if (_t <= t2_) {
    return -am_;
  }
  else if (_t <= t3_) {
    return a(t2_) + jm_ * (_t - t2_);
  }
  else {
    return 0;
  }
}
}  // namespace tobas::mission
