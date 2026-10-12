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
  const kdl::Rotation& _cur_rot,
  const kdl::Vector& _cur_gyro,
  const kdl::Vector& _cur_dgyro,
  const kdl::Rotation& _tar_rot,
  const kdl::Vector& _tar_gyro,
  const kdl::Vector& _tar_dgyro,
  double _dt)
{
  // Compute error in angle-axis form wrt. the local frame.
  const auto ep = (_cur_rot.inverse() * _tar_rot).getRot();
  const auto ev = _tar_gyro - _cur_gyro;
  const auto ea = _tar_dgyro - _cur_dgyro;

  // Compute command ddgyro.
  const auto cmd_ddgyro = kp_.hadamard(ep) + kv_.hadamard(ev) + ka_.hadamard(ea);

  // Integrate command ddgyro.
  cmd_dgyro_ += cmd_ddgyro * _dt;
  return cmd_dgyro_;
}

void AngleAxisPDD2::setNaturalFreq(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  wn_(_idx) = _value;
  updateGain();
}

void AngleAxisPDD2::setInertiaRatio(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  zeta_(_idx) = _value;
  updateGain();
}

void AngleAxisPDD2::setDampingRatio(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  xi_(_idx) = _value;
  updateGain();
}

void AngleAxisPDD2::updateGain()
{
  kp_ = wn_.cube();
  kv_ = 3 * xi_.sqr().hadamard(wn_.sqr());
  ka_ = 3 * zeta_.hadamard(xi_).hadamard(wn_);
}
}  // namespace tobas
