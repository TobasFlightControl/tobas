// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas
{
namespace traj
{
/* Linear trajectory generation. */
class LinearSpline : public TrajectoryGenerator
{
public:
  explicit LinearSpline(double p0, double pf, double T);

  TrajectoryPoint get(double t) const noexcept override;
  double duration() const noexcept override;

private:
  const double p0_, T_;
  double v_;
};
}  // namespace traj
}  // namespace tobas
