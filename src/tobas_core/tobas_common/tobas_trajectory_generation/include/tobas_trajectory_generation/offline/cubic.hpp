// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas::traj
{
/** Cubic polynomial trajectory generation. (Introduction to ROBOTICS, p.192) */
class CubicSpline : public TrajectoryGenerator
{
public:
  explicit CubicSpline(double _p0, double _pf, double _t);

  TrajectoryPoint get(double _t) const noexcept override;
  double duration() const noexcept override;

private:
  const double T_;
  double a0_, a1_, a2_, a3_;
};
}  // namespace tobas::traj
