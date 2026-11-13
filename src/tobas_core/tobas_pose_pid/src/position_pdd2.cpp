// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_pose_pid/position_pdd2.hpp"

#include <cassert>

namespace tobas
{
PositionPDD2::PositionPDD2()
{
  updateGain();
}

kdl::Vector PositionPDD2::update(
  const kdl::Vector& _cur_pos,
  const kdl::Vector& _cur_vel,
  const kdl::Vector& _cur_acc,
  const kdl::Vector& _tar_pos,
  const kdl::Vector& _tar_vel,
  const kdl::Vector& _tar_acc,
  double _dt)
{
  // Calculate errors.
  const auto ep = _tar_pos - _cur_pos;
  const auto ev = _tar_vel - _cur_vel;
  const auto ea = _tar_acc - _cur_acc;

  // PDD2 + clamp
  const auto cmd_jerk = kp_.hadamard(ep) + kv_.hadamard(ev) + ka_.hadamard(ea);

  // Integrate the target jerk and output it.
  cmd_acc_ += cmd_jerk * _dt;
  return cmd_acc_;
}

void PositionPDD2::setNaturalFreq(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  wn_(_idx) = _value;
  updateGain();
}

void PositionPDD2::setInertiaRatio(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  zeta_(_idx) = _value;
  updateGain();
}

void PositionPDD2::setDampingRatio(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  xi_(_idx) = _value;
  updateGain();
}

void PositionPDD2::updateGain()
{
  kp_ = wn_.cube();
  kv_ = 3 * xi_.sqr().hadamard(wn_.sqr());
  ka_ = 3 * zeta_.hadamard(xi_).hadamard(wn_);
}
}  // namespace tobas
