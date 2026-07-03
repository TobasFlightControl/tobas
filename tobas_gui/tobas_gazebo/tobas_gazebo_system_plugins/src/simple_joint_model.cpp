// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_system_plugins/simple_joint_model.hpp"

#include <gz/common/Console.hh>

#define POS_MARGIN 1e-2  // [rad]

namespace tobas
{
namespace gazebo
{
SimpleJointModel::SimpleJointModel(double _min_pos, double _max_pos, double _max_vel)
  : pos_limit(_min_pos, _max_pos), max_vel(_max_vel)
{
  assert(pos_limit.isValid());
  assert(max_vel >= 0.);
}

SimpleJointModel::SimpleJointModel() : SimpleJointModel(0., 0., 0.)
{
}

double SimpleJointModel::getCurrentPosition() const
{
  return cur_pos_;
}

void SimpleJointModel::setTargetPosition(double tar_pos)
{
  if (pos_limit.inRange(tar_pos, POS_MARGIN)) {
    tar_pos_ = tar_pos;
  }
  else {
    gzwarn << "Target joint position is out of range: " << tar_pos << " ∉ " << pos_limit << std::endl;
    tar_pos_ = pos_limit.clamp(tar_pos);
  }
}

void SimpleJointModel::step(double dt)
{
  assert(dt >= 0.);

  // Velocity limit.
  const auto ideal_delta_angle = tar_pos_ - cur_pos_;
  const auto max_delta_angle = max_vel * dt;
  const auto delta_angle = std::clamp(ideal_delta_angle, -max_delta_angle, max_delta_angle);

  // Position limit.
  const auto cnd_angle = cur_pos_ + delta_angle;
  cur_pos_ = pos_limit.clamp(cnd_angle);
}
}  // namespace gazebo
}  // namespace tobas
