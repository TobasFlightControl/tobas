// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas::traj
{
/** Linear trajectory generation. */
class LinearSpline : public TrajectoryGenerator
{
public:
  explicit LinearSpline(double _p0, double _pf, double _t);

  TrajectoryPoint get(double _t) const noexcept override;
  double duration() const noexcept override;

private:
  const double p0_, T_;
  double v_;
};
}  // namespace tobas::traj
