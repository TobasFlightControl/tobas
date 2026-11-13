// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_pose_pid/angle_axis_pid.hpp"

#include <cassert>

namespace tobas
{
AngleAxisPID::AngleAxisPID()
{
  updateGain();
}

kdl::Vector AngleAxisPID::update(
  const kdl::Rotation& _cur_rot,
  const kdl::Vector& _cur_gyro,
  const kdl::Rotation& _tar_rot,
  const kdl::Vector& _tar_gyro,
  double _dt)
{
  // Compute error in angle-axis form wrt. the local frame.
  const auto ep = (_cur_rot.inverse() * _tar_rot).getRot();
  const auto ed = _tar_gyro - _cur_gyro;

  // Integrate error.
  for (size_t i = 0; i < 3; ++i) {
    if (ki_(i) > 0.0) {
      ei_(i) += ep(i) * _dt;
    }
  }

  // Compute target dgyro.
  return kp_.hadamard(ep) + ki_.hadamard(ei_) + kd_.hadamard(ed);
}

void AngleAxisPID::setNaturalFreq(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  natural_freq_(_idx) = _value;
  updateGain();
}

void AngleAxisPID::setDampingRatio(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  damp_ratio_(_idx) = _value;
  updateGain();
}

void AngleAxisPID::setIntegralGain(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  ki_(_idx) = _value;
  ei_(_idx) = 0.0;
}

void AngleAxisPID::updateGain()
{
  kp_ = natural_freq_.sqr();
  kd_ = 2 * damp_ratio_.hadamard(natural_freq_);
}
}  // namespace tobas
