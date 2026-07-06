// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_std_tools/range.hpp>

namespace tobas
{
namespace gazebo
{
/* Simple joint model with position and maximum-velocity constraints. */
class SimpleJointModel
{
public:
  st::Range<double> pos_limit;
  double max_vel;

  explicit SimpleJointModel(double _min_pos, double _max_pos, double _max_vel);
  explicit SimpleJointModel();

  double getCurrentPosition() const;
  void setTargetPosition(double tar_pos);
  void step(double dt);

private:
  double cur_pos_ = 0.0;
  double tar_pos_ = 0.0;
};
}  // namespace gazebo
}  // namespace tobas
