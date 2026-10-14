// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_pose_pid/position_pid.hpp"

#include <cassert>

namespace tobas
{
PositionPID::PositionPID()
{
}

kdl::Vector PositionPID::update(
  const kdl::Vector& _cur_pos,
  const kdl::Vector& _cur_vel,
  const kdl::Vector& _tar_pos,
  const kdl::Vector& _tar_vel,
  double _dt)
{
  // Calculate errors.
  const auto ep = _tar_pos - _cur_pos;
  const auto ed = _tar_vel - _cur_vel;

  for (size_t i = 0; i < 3; ++i) {
    if (ki_(i) > 0.0) {  // When using integral control
      // Accumulate integral error.
      const auto next_ei = ei_(i) + ep(i) * _dt;
      ei_(i) = std::clamp(next_ei, -max_i_acc_(i), max_i_acc_(i));
    }
    else  // When not using integral control
    {
      // Reset integral error.
      ei_(i) = 0.0;
    }
  }

  // Calculate target acceleration.
  return kp_.hadamard(ep) + ki_.hadamard(ei_) + kd_.hadamard(ed);
}

void PositionPID::setProportionalGain(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  kp_(_idx) = _value;
}

void PositionPID::setIntegralGain(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  ki_(_idx) = _value;
}

void PositionPID::setDerivativeGain(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  kd_(_idx) = _value;
}

void PositionPID::setNaturalFreq(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  natural_freq_(_idx) = _value;
  setGainFromSecondOrderFrom();
}

void PositionPID::setDampingRatio(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  damp_ratio_(_idx) = _value;
  setGainFromSecondOrderFrom();
}

void PositionPID::setMaxIntegralAccel(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  max_i_acc_(_idx) = _value;
}

void PositionPID::setGainFromSecondOrderFrom()
{
  kp_ = natural_freq_.sqr();
  kd_ = 2 * damp_ratio_.hadamard(natural_freq_);
}
}  // namespace tobas
