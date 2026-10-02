// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_pose_pid/angle_axis_pdd2.hpp"

#include <cassert>

namespace tobas
{
AngleAxisPDD2::AngleAxisPDD2()
{
  updateGain();
}

kdl::Vector AngleAxisPDD2::update(
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_gyro,
  const kdl::Vector& cur_dgyro,
  const kdl::Rotation& tar_rot,
  const kdl::Vector& tar_gyro,
  const kdl::Vector& tar_dgyro,
  double dt)
{
  // Compute error in angle-axis form wrt. the local frame.
  const auto ep = (cur_rot.inverse() * tar_rot).getRot();
  const auto ev = tar_gyro - cur_gyro;
  const auto ea = tar_dgyro - cur_dgyro;

  // Compute command ddgyro.
  const auto cmd_ddgyro = kp_.hadamard(ep) + kv_.hadamard(ev) + ka_.hadamard(ea);

  // Integrate command ddgyro.
  cmd_dgyro_ += cmd_ddgyro * dt;
  return cmd_dgyro_;
}

void AngleAxisPDD2::setNaturalFreq(int idx, double value)
{
  assert(0 <= idx && idx < 3);
  assert(value >= 0.0);

  wn_(idx) = value;
  updateGain();
}

void AngleAxisPDD2::setInertiaRatio(int idx, double value)
{
  assert(0 <= idx && idx < 3);
  assert(value >= 0.0);

  zeta_(idx) = value;
  updateGain();
}

void AngleAxisPDD2::setDampingRatio(int idx, double value)
{
  assert(0 <= idx && idx < 3);
  assert(value >= 0.0);

  xi_(idx) = value;
  updateGain();
}

void AngleAxisPDD2::updateGain()
{
  kp_ = wn_.cube();
  kv_ = 3 * xi_.sqr().hadamard(wn_.sqr());
  ka_ = 3 * zeta_.hadamard(xi_).hadamard(wn_);
}
}  // namespace tobas
