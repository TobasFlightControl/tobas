// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>

namespace tobas
{
namespace traj
{
/* memo: 3-50 */
class AccelLimitedOnlineTrajectoryGenerator
{
public:
  explicit AccelLimitedOnlineTrajectoryGenerator();

  inline double getTrajectoryPosition() const;
  inline double getTrajectoryVelocity() const;

  inline void setTargetPosition(double tar_pos);
  inline void setTargetVelocity(double tar_vel);

  void setMaxAccel(double max_acc);

  void update(double dt);

  void setTargetPointAndUpdate(double tar_pos, double tar_vel, double dt);

  void resetCurrentTrajectoryPoint(double pos, double vel);

private:
  // Trajectory Point
  double p_ = 0.;
  double v_ = 0.;

  // Target
  double pf_ = 0.;
  double vf_ = 0.;

  // Limit
  double am_ = NAN;
};

inline double AccelLimitedOnlineTrajectoryGenerator::getTrajectoryPosition() const
{
  return p_;
}

inline double AccelLimitedOnlineTrajectoryGenerator::getTrajectoryVelocity() const
{
  return v_;
}

inline void AccelLimitedOnlineTrajectoryGenerator::setTargetPosition(double tar_pos)
{
  pf_ = tar_pos;
}

inline void AccelLimitedOnlineTrajectoryGenerator::setTargetVelocity(double tar_vel)
{
  vf_ = tar_vel;
}
}  // namespace traj
}  // namespace tobas
