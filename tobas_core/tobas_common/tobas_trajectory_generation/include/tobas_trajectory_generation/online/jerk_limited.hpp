// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>

namespace tobas
{
namespace traj
{
/**
 * Online update of the minimum-time trajectory that reaches the target state while satisfying constraints.
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

  inline void setTargetPosition(double _tar_pos);
  inline void setTargetVelocity(double _tar_vel);
  inline void setTargetAcceleration(double _tar_acc);

  void setMinVelocity(double _min_vel);
  void setMaxVelocity(double _max_vel);
  void setMinAcceleration(double _min_acc);
  void setMaxAcceleration(double _max_acc);
  void setMaxJerk(double _max_jerk);

  void update(double _dt);

  void setTargetPointAndUpdate(double _tar_pos, double _tar_vel, double _tar_acc, double _dt);

  void resetCurrentTrajectoryPoint(double _pos, double _vel, double _acc);

private:
  // Trajectory Point
  double traj_pos_ = 0.0;
  double traj_vel_ = 0.0;
  double traj_acc_ = 0.0;

  // Target
  double tar_pos_ = 0.0;
  double tar_vel_ = 0.0;
  double tar_acc_ = 0.0;

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

inline void JerkLimitedOnlineTrajectoryGenerator::setTargetPosition(double _tar_pos)
{
  tar_pos_ = _tar_pos;
}

inline void JerkLimitedOnlineTrajectoryGenerator::setTargetVelocity(double _tar_vel)
{
  tar_vel_ = _tar_vel;
}

inline void JerkLimitedOnlineTrajectoryGenerator::setTargetAcceleration(double _tar_acc)
{
  tar_acc_ = _tar_acc;
}
}  // namespace traj
}  // namespace tobas
