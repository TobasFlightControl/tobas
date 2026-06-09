// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_trajectory_generation/offline/time_optimal.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>

#include <tobas_math/core.hpp>

#define EPS 1e-6  // 小さすぎると永久に収束しない恐れがある

namespace tobas
{
namespace traj
{
TimeOptimalTrajectory::TimeOptimalTrajectory(double p0, double pf, double max_jerk, double max_acc, double max_vel)
  : p0_(p0), pd_(std::abs(pf - p0)), sign_(math::sign(pf - p0)), jm_(max_jerk), am_(max_acc), vm_(max_vel)
{
  assert(jm_ > 0.);
  assert(am_ > 0.);
  assert(vm_ > 0.);

  // 開始位置と目標位置が一致している場合は例外
  if (pd_ < EPS) {
    t1_ = t2_ = t3_ = t4_ = 0.;
    return;
  }

  // 時刻の大小関係の制約を満たすように加速度と速度の最大値を調整
  bool ok = false;
  size_t iter = 0;
  while (!ok) {
    ok = true;
    ++iter;

    // 最大加速度に到達するための条件
    if (math::sqr(am_) > vm_ * jm_) {
      am_ = std::sqrt(vm_ * jm_) - EPS;
      ok = false;
    }

    // 最大速度に到達するための条件
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

TrajectoryPoint TimeOptimalTrajectory::get(double t) const noexcept
{
  if (pd_ < EPS) {
    return { p0_, 0., 0. };
  }

  // 最初に除いた原点と移動方向を反映
  return { p0_ + sign_ * p(t), sign_ * v(t), sign_ * a(t) };
}

double TimeOptimalTrajectory::duration() const noexcept
{
  return 2 * t4_;
}

double TimeOptimalTrajectory::p(double t) const noexcept
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
    return pd_ - p(2 * t4_ - t);  // t = t4, p = pd/2 に関する点対称性を利用
  }
}

double TimeOptimalTrajectory::v(double t) const noexcept
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
    return vm_ - v(t3_ - t);  // t = t3/2, v = vm/2 に関する点対称性を利用
  }
  else if (t <= t4_) {
    return vm_;
  }
  else {
    return v(2 * t4_ - t);  // t = t4 に関する線対称性を利用
  }
}

double TimeOptimalTrajectory::a(double t) const noexcept
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
    return a(t3_ - t);  // t = t3/2 に関する線対称性を利用
  }
  else if (t <= t4_) {
    return 0;
  }
  else {
    return -a(2 * t4_ - t);  // t = t4, a = 0 に関する点対称性を利用
  }
}
}  // namespace traj
}  // namespace tobas
