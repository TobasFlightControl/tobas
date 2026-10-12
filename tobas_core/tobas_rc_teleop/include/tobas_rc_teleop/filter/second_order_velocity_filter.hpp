// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <limits>

namespace tobas
{
namespace rc
{
class SecondOrderVelocityFilter
{
public:
  explicit SecondOrderVelocityFilter();

  inline double getTrajectoryVelocity() const;
  inline double getTrajectoryAccel() const;

  void setMaxVelocity(double _v_max);
  void setMaxJerk(double _j_max);

  void update(double _v_des, double _dt);

  void resetCurrentTrajectoryPoint(double _v, double _a);

private:
  // State
  double v_ = 0.0;
  double a_ = 0.0;

  // Config
  double v_max_ = std::numeric_limits<double>::max();
  double j_max_ = std::numeric_limits<double>::max();
};

inline double SecondOrderVelocityFilter::getTrajectoryVelocity() const
{
  return v_;
}

inline double SecondOrderVelocityFilter::getTrajectoryAccel() const
{
  return a_;
}
}  // namespace rc
}  // namespace tobas
