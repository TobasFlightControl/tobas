// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_trajectory_generation/online/velocity_limited.hpp"

#include <cassert>
#include <cmath>

namespace tobas::traj
{
VelocityLimitedOnlineTrajectoryGenerator::VelocityLimitedOnlineTrajectoryGenerator()
{
}

void VelocityLimitedOnlineTrajectoryGenerator::setMaxVelocity(double _max_vel)
{
  assert(_max_vel > 0);
  max_vel_ = _max_vel;
}

double VelocityLimitedOnlineTrajectoryGenerator::update(double _dt)
{
  assert(std::isfinite(max_vel_));
  assert(_dt >= 0);

  const auto max_delta = max_vel_ * _dt;

  auto delta = tar_pos_ - traj_pos_;
  if (delta > max_delta) {
    delta = max_delta;
    is_saturated_ = true;
  }
  else if (delta < -max_delta) {
    delta = -max_delta;
    is_saturated_ = true;
  }
  else {
    is_saturated_ = false;
  }

  traj_pos_ += delta;

  return traj_pos_;
}

double VelocityLimitedOnlineTrajectoryGenerator::setTargetPointAndUpdate(double _tar_pos, double _dt)
{
  setTargetPosition(_tar_pos);
  return update(_dt);
}

void VelocityLimitedOnlineTrajectoryGenerator::resetCurrentTrajectoryPoint(double _pos)
{
  traj_pos_ = _pos;
}
}  // namespace tobas::traj
