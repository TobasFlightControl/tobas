// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_pose_pid/euler_pid.hpp"

#include <cassert>

#include <tobas_algorithm/core.hpp>
#include <tobas_eigen_tools/kinematics.hpp>

namespace tobas
{
EulerPID::EulerPID()
{
  updateGain();
}

kdl::Vector EulerPID::update(
  const kdl::Euler& _cur_rpy,
  const kdl::Vector& _cur_gyro,
  const kdl::Euler& _tar_rpy,
  const kdl::Vector& _tar_gyro,
  double _dt)
{
  // Calculate errors.
  const auto ep = computeProportionalError(_cur_rpy, _tar_rpy);
  const auto ed = computeDerivativeError(_cur_rpy, _cur_gyro, _tar_gyro);

  // Accumulate integral error when using integral control.
  for (size_t i = 0; i < 3; ++i) {
    if (ki_(i) > 0.0) {
      ei_(i) += ep(i) * _dt;
    }
  }

  // Calculate target Euler angle acceleration.
  const auto tar_ddrpy = kp_.hadamard(ep) + ki_.hadamard(ei_) + kd_.hadamard(ed);

  // Convert Euler angle acceleration to differential gyro.
  const auto cur_drpy = eigen::eulerrateFromAngvelLocal(_cur_gyro.data, _cur_rpy.roll, _cur_rpy.pitch);
  return eigen::angaccFromEuleraccLocal(_cur_rpy.roll, _cur_rpy.pitch, cur_drpy, tar_ddrpy.data);
}

void EulerPID::setNaturalFreq(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  natural_freq_(_idx) = _value;
  updateGain();
}

void EulerPID::setDampingRatio(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  damp_ratio_(_idx) = _value;
  updateGain();
}

void EulerPID::setIntegralGain(int _idx, double _value)
{
  assert(0 <= _idx && _idx < 3);
  assert(_value >= 0.0);

  ki_(_idx) = _value;
  ei_(_idx) = 0.0;
}

void EulerPID::updateGain()
{
  kp_ = natural_freq_.sqr();
  kd_ = 2 * damp_ratio_.hadamard(natural_freq_);
}

kdl::Vector EulerPID::computeProportionalError(const kdl::Euler& _cur_rpy, const kdl::Euler& _tar_rpy)
{
  // Note that a straight line between two Euler angles is not the shortest rotation path.
  const auto roll_err = algo::wrapPi(_tar_rpy.roll - _cur_rpy.roll);
  const auto pitch_err = algo::wrapPi(_tar_rpy.pitch - _cur_rpy.pitch);
  const auto yaw_err = algo::wrapPi(_tar_rpy.yaw - _cur_rpy.yaw);
  return { roll_err, pitch_err, yaw_err };
}

kdl::Vector
EulerPID::computeDerivativeError(const kdl::Euler& _cur_rpy, const kdl::Vector& _cur_gyro, const kdl::Vector& _tar_gyro)
{
  const auto gyro_error = _tar_gyro - _cur_gyro;
  return eigen::eulerrateFromAngvelLocal(gyro_error.data, _cur_rpy.roll, _cur_rpy.pitch);
}
}  // namespace tobas
