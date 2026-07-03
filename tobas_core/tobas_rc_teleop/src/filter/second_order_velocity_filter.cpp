// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/filter/second_order_velocity_filter.hpp"

#include <algorithm>
#include <cassert>

#include <tobas_math/core.hpp>

namespace tobas
{
namespace rc
{
SecondOrderVelocityFilter::SecondOrderVelocityFilter()
{
}

void SecondOrderVelocityFilter::setMaxVelocity(double v_max)
{
  assert(v_max > 0);
  v_max_ = v_max;
}

void SecondOrderVelocityFilter::setMaxJerk(double j_max)
{
  assert(j_max > 0);
  j_max_ = j_max;
}

void SecondOrderVelocityFilter::update(double v_des, double dt)
{
  assert(dt >= 0);

  const auto wn = std::sqrt(j_max_ / v_max_);  // Set max jerk when the velocity error is `v_max`.
  const auto kp = math::sqr(wn);
  const auto kd = 2 * wn;  // Critical damping: zeta = 1

  const auto j = std::clamp(kp * (v_des - v_) + kd * (0 - a_), -j_max_, j_max_);

  v_ += a_ * dt;
  a_ += j * dt;
}

void SecondOrderVelocityFilter::resetCurrentTrajectoryPoint(double v, double a)
{
  v_ = v;
  a_ = a;
}
}  // namespace rc
}  // namespace tobas
