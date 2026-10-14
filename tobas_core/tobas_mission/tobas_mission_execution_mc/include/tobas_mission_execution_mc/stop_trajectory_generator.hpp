// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_trajectory_generation/offline/base.hpp>

namespace tobas
{
namespace mission
{
/** Fastest stopping trajectory from an arbitrary velocity and acceleration (memo: 3-49) */
class StopTrajectory : public traj::TrajectoryGenerator
{
public:
  explicit StopTrajectory(double _p0, double _v0, double _a0, double _am, double _jm);

  traj::TrajectoryPoint get(double _t) const noexcept override;
  double duration() const noexcept override;

private:
  const double p0_, v0_, a0_;
  double am_, jm_;
  double t1_, t2_, t3_;

  double p(double _t) const noexcept;
  double v(double _t) const noexcept;
  double a(double _t) const noexcept;
};
}  // namespace mission
}  // namespace tobas
