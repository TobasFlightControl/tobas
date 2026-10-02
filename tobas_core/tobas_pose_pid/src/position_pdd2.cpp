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
  const kdl::Vector& cur_pos,
  const kdl::Vector& cur_vel,
  const kdl::Vector& cur_acc,
  const kdl::Vector& tar_pos,
  const kdl::Vector& tar_vel,
  const kdl::Vector& tar_acc,
  double dt)
{
  // Calculate errors.
  const auto ep = tar_pos - cur_pos;
  const auto ev = tar_vel - cur_vel;
  const auto ea = tar_acc - cur_acc;

  // PDD2 + clamp
  const auto cmd_jerk = kp_.hadamard(ep) + kv_.hadamard(ev) + ka_.hadamard(ea);

  // Integrate the target jerk and output it.
  cmd_acc_ += cmd_jerk * dt;
  return cmd_acc_;
}

void PositionPDD2::setNaturalFreq(int idx, double value)
{
  assert(0 <= idx && idx < 3);
  assert(value >= 0.0);

  wn_(idx) = value;
  updateGain();
}

void PositionPDD2::setInertiaRatio(int idx, double value)
{
  assert(0 <= idx && idx < 3);
  assert(value >= 0.0);

  zeta_(idx) = value;
  updateGain();
}

void PositionPDD2::setDampingRatio(int idx, double value)
{
  assert(0 <= idx && idx < 3);
  assert(value >= 0.0);

  xi_(idx) = value;
  updateGain();
}

void PositionPDD2::updateGain()
{
  kp_ = wn_.cube();
  kv_ = 3 * xi_.sqr().hadamard(wn_.sqr());
  ka_ = 3 * zeta_.hadamard(xi_).hadamard(wn_);
}
}  // namespace tobas
