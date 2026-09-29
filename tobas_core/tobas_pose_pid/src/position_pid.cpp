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
  const kdl::Vector& cur_pos,
  const kdl::Vector& cur_vel,
  const kdl::Vector& tar_pos,
  const kdl::Vector& tar_vel,
  const double& dt)
{
  // Calculate errors.
  const auto ep = tar_pos - cur_pos;
  const auto ed = tar_vel - cur_vel;

  for (size_t i = 0; i < 3; ++i) {
    if (ki_(i) > 0.0) {  // When using integral control
      // Accumulate integral error.
      const auto next_ei = ei_(i) + ep(i) * dt;
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

void PositionPID::setProportionalGain(int idx, double value)
{
  assert(0 <= idx && idx < 3);
  assert(value >= 0.0);

  kp_(idx) = value;
}

void PositionPID::setIntegralGain(int idx, double value)
{
  assert(0 <= idx && idx < 3);
  assert(value >= 0.0);

  ki_(idx) = value;
}

void PositionPID::setDerivativeGain(int idx, double value)
{
  assert(0 <= idx && idx < 3);
  assert(value >= 0.0);

  kd_(idx) = value;
}

void PositionPID::setNaturalFreq(int idx, double value)
{
  assert(0 <= idx && idx < 3);
  assert(value >= 0.0);

  natural_freq_(idx) = value;
  setGainFromSecondOrderFrom();
}

void PositionPID::setDampingRatio(int idx, double value)
{
  assert(0 <= idx && idx < 3);
  assert(value >= 0.0);

  damp_ratio_(idx) = value;
  setGainFromSecondOrderFrom();
}

void PositionPID::setMaxIntegralAccel(int idx, double value)
{
  assert(0 <= idx && idx < 3);
  assert(value >= 0.0);

  max_i_acc_(idx) = value;
}

void PositionPID::setGainFromSecondOrderFrom()
{
  kp_ = natural_freq_.sqr();
  kd_ = 2 * damp_ratio_.hadamard(natural_freq_);
}
}  // namespace tobas
