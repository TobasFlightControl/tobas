// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_mission_execution_mc/stop_trajectory_generator.hpp"

#include <cassert>

#include <tobas_math/core.hpp>

#define EPS 1e-6

namespace tobas
{
namespace mission
{
StopTrajectory::StopTrajectory(double p0, double v0, double a0, double am, double jm)
  : p0_(p0), v0_(v0), a0_(a0), am_(am), jm_(jm)
{
  assert(v0_ >= 0.);
  assert(am_ > 0.);
  assert(jm_ > 0.);

  // 既に停止している場合は例外
  if (v0_ < EPS) {
    t1_ = t2_ = t3_ = 0.;
    return;
  }

  // 時刻の大小関係の制約を満たすようにジャークと加速度の最大値を調整
  if (a0_ < 0) {
    jm_ = std::max(jm_, math::sqr(a0_) / (2 * v0_) + EPS);
    am_ = std::max(am_, -a0_);
  }
  am_ = std::min(am_, sqrt(math::sqr(a0_) / 2 + v0_ * jm_) - EPS);

  t1_ = (a0_ + am_) / jm_;
  t2_ = math::sqr(a0_) / (2 * am_ * jm_) + v0_ / am_ + a0_ / jm_;
  t3_ = t2_ + am_ / jm_;

  assert(t1_ < t2_);
  assert(t2_ < t3_);
}

traj::TrajectoryPoint StopTrajectory::get(double t) const noexcept
{
  if (v0_ < EPS) {
    return { p0_, 0., 0. };
  }

  return { p(t), v(t), a(t) };
}

double StopTrajectory::duration() const noexcept
{
  return t3_;
}

double StopTrajectory::p(double t) const noexcept
{
  if (t <= 0) {
    return p0_;
  }
  else if (t <= t1_) {
    return p0_ + v0_ * t + (a0_ / 2) * math::sqr(t) - (jm_ / 6) * math::cube(t);
  }
  else if (t <= t2_) {
    const auto dt = t - t1_;
    return p(t1_) + v(t1_) * dt - (am_ / 2) * math::sqr(dt);
  }
  else if (t <= t3_) {
    const auto dt = t - t2_;
    return p(t2_) + v(t2_) * dt + (a(t2_) / 2) * math::sqr(dt) + (jm_ / 6) * math::cube(dt);
  }
  else {
    return p(t3_);
  }
}

double StopTrajectory::v(double t) const noexcept
{
  if (t <= 0) {
    return v0_;
  }
  else if (t <= t1_) {
    return v0_ + a0_ * t - (jm_ / 2) * math::sqr(t);
  }
  else if (t <= t2_) {
    return v(t1_) - am_ * (t - t1_);
  }
  else if (t <= t3_) {
    const auto dt = t - t2_;
    return v(t2_) + a(t2_) * dt + (jm_ / 2) * math::sqr(dt);
  }
  else {
    return 0;
  }
}

double StopTrajectory::a(double t) const noexcept
{
  if (t <= 0) {
    return a0_;
  }
  else if (t <= t1_) {
    return a0_ - jm_ * t;
  }
  else if (t <= t2_) {
    return -am_;
  }
  else if (t <= t3_) {
    return a(t2_) + jm_ * (t - t2_);
  }
  else {
    return 0;
  }
}
}  // namespace mission
}  // namespace tobas
