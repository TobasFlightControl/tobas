// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas
{
namespace traj
{
/* Minimum-time trajectory to the destination while satisfying acceleration and velocity constraints. (memo: 3-50) */
class AccelLimitedTrajectory : public TrajectoryGenerator
{
public:
  explicit AccelLimitedTrajectory(double p0, double v0, double pf, double vf, double max_acc);

  TrajectoryPoint get(double t) const noexcept override;
  double duration() const noexcept override;

private:
  const double p0_, v0_;  // Initial state
  const double pf_, vf_;  // Target state
  const double am_;       // Limit

  double s_;   // Switching curve
  double ts_;  // Switching time
  double tf_;  // Arrival time
};
}  // namespace traj
}  // namespace tobas
