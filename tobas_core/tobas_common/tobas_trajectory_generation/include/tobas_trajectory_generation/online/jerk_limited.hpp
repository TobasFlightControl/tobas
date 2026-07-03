// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>

namespace tobas
{
namespace traj
{
/**
 * @brief Online update of the minimum-time trajectory that reaches the target state while satisfying constraints.
 *
 * @note Bang-bang control has no stability margin,
 * so it is difficult to use as a feedback loop in a real environment with delays and modeling errors.
 */
class JerkLimitedOnlineTrajectoryGenerator
{
public:
  explicit JerkLimitedOnlineTrajectoryGenerator();

  inline double getTrajectoryPosition() const;
  inline double getTrajectoryVelocity() const;
  inline double getTrajectoryAcceleration() const;

  inline void setTargetPosition(double tar_pos);
  inline void setTargetVelocity(double tar_vel);
  inline void setTargetAcceleration(double tar_acc);

  void setMinVelocity(double min_vel);
  void setMaxVelocity(double max_vel);
  void setMinAcceleration(double min_acc);
  void setMaxAcceleration(double max_acc);
  void setMaxJerk(double max_jerk);

  void update(double dt);

  void setTargetPointAndUpdate(double tar_pos, double tar_vel, double tar_acc, double dt);

  void resetCurrentTrajectoryPoint(double pos, double vel, double acc);

private:
  // Trajectory Point
  double traj_pos_ = 0.;
  double traj_vel_ = 0.;
  double traj_acc_ = 0.;

  // Target
  double tar_pos_ = 0.;
  double tar_vel_ = 0.;
  double tar_acc_ = 0.;

  // Limit
  double v_min_ = NAN;
  double v_max_ = NAN;
  double a_min_ = NAN;
  double a_max_ = NAN;
  double u_ = NAN;
};

inline double JerkLimitedOnlineTrajectoryGenerator::getTrajectoryPosition() const
{
  return traj_pos_;
}

inline double JerkLimitedOnlineTrajectoryGenerator::getTrajectoryVelocity() const
{
  return traj_vel_;
}

inline double JerkLimitedOnlineTrajectoryGenerator::getTrajectoryAcceleration() const
{
  return traj_acc_;
}

inline void JerkLimitedOnlineTrajectoryGenerator::setTargetPosition(double tar_pos)
{
  tar_pos_ = tar_pos;
}

inline void JerkLimitedOnlineTrajectoryGenerator::setTargetVelocity(double tar_vel)
{
  tar_vel_ = tar_vel;
}

inline void JerkLimitedOnlineTrajectoryGenerator::setTargetAcceleration(double tar_acc)
{
  tar_acc_ = tar_acc;
}
}  // namespace traj
}  // namespace tobas
