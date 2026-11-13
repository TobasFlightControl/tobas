// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>

namespace tobas::traj
{
class VelocityLimitedOnlineTrajectoryGenerator
{
public:
  explicit VelocityLimitedOnlineTrajectoryGenerator();

  inline bool isSaturated() const;

  inline double getTrajectoryPosition() const;

  inline void setTargetPosition(double _tar_pos);

  void setMaxVelocity(double _max_vel);

  double update(double _dt);

  double setTargetPointAndUpdate(double _tar_pos, double _dt);

  void resetCurrentTrajectoryPoint(double _pos);

private:
  // State
  double traj_pos_ = 0.0;
  bool is_saturated_ = false;

  // Target
  double tar_pos_ = 0.0;

  // Limit
  double max_vel_ = NAN;
};

inline bool VelocityLimitedOnlineTrajectoryGenerator::isSaturated() const
{
  return is_saturated_;
}

inline double VelocityLimitedOnlineTrajectoryGenerator::getTrajectoryPosition() const
{
  return traj_pos_;
}

inline void VelocityLimitedOnlineTrajectoryGenerator::setTargetPosition(double _tar_pos)
{
  tar_pos_ = _tar_pos;
}
}  // namespace tobas::traj
