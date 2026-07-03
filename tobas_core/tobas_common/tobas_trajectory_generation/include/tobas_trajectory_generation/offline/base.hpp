// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace traj
{
struct TrajectoryPoint
{
  double p;
  double v;
  double a;
};

/* Base class for trajectory generators. */
class TrajectoryGenerator
{
public:
  virtual TrajectoryPoint get(double t) const noexcept = 0;
  virtual double duration() const noexcept = 0;
};
}  // namespace traj
}  // namespace tobas
