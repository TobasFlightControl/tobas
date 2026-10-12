// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas
{
namespace traj
{
/**
 * Minimum-time trajectory to the destination while satisfying jerk, acceleration, and velocity constraints.
 * (memo: 3-44)
 */
class JerkLimitedTrajectory : public TrajectoryGenerator
{
public:
  explicit JerkLimitedTrajectory(double _p0, double _pf, double _max_jerk, double _max_acc, double _max_vel);

  TrajectoryPoint get(double _t) const noexcept override;
  double duration() const noexcept override;

private:
  const double p0_;           ///< Initial position
  const double pd_;           ///< Destination with `p0` as the origin, unsigned
  const int sign_;            ///< Sign of `pd`
  double jm_, am_, vm_;       ///< Limits
  double t1_, t2_, t3_, t4_;  ///< Times

  /** Relative position with respect to p0 when pd is positive. */
  double p(double _t) const noexcept;
  /** Velocity when pd is positive. */
  double v(double _t) const noexcept;
  /** Acceleration when pd is positive. */
  double a(double _t) const noexcept;
};
}  // namespace traj
}  // namespace tobas
