// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_system_plugins/simple_joint_model.hpp"

#include <gz/common/Console.hh>

namespace tobas::gazebo
{
SimpleJointModel::SimpleJointModel(double _min_pos, double _max_pos, double _max_vel)
  : pos_limit(_min_pos, _max_pos), max_vel(_max_vel)
{
  assert(pos_limit.isValid());
  assert(max_vel >= 0.0);
}

SimpleJointModel::SimpleJointModel() : SimpleJointModel(0.0, 0.0, 0.0)
{
}

double SimpleJointModel::getCurrentPosition() const
{
  return cur_pos_;
}

void SimpleJointModel::setTargetPosition(double _tar_pos)
{
  constexpr auto kPositionMargin = 1e-2;  // [rad]
  if (pos_limit.inRange(_tar_pos, kPositionMargin)) {
    tar_pos_ = _tar_pos;
  }
  else {
    gzwarn << "Target joint position is out of range: " << _tar_pos << " ∉ " << pos_limit << '\n';
    tar_pos_ = pos_limit.clamp(_tar_pos);
  }
}

void SimpleJointModel::step(double _dt)
{
  assert(_dt >= 0.0);

  // Velocity limit.
  const auto ideal_delta_angle = tar_pos_ - cur_pos_;
  const auto max_delta_angle = max_vel * _dt;
  const auto delta_angle = std::clamp(ideal_delta_angle, -max_delta_angle, max_delta_angle);

  // Position limit.
  const auto cnd_angle = cur_pos_ + delta_angle;
  cur_pos_ = pos_limit.clamp(cnd_angle);
}
}  // namespace tobas::gazebo
