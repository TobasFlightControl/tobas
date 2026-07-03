// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas
{
namespace traj
{
/* Cubic polynomial trajectory generation. (Robotics, p.192) */
class CubicSpline : public TrajectoryGenerator
{
public:
  explicit CubicSpline(double p0, double pf, double T);

  TrajectoryPoint get(double t) const noexcept override;
  double duration() const noexcept override;

private:
  const double T_;
  double a0_, a1_, a2_, a3_;
};
}  // namespace traj
}  // namespace tobas
