// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_pose_pid/euler_pid.hpp"

#include <tobas_algorithm/core.hpp>
#include <tobas_eigen_tools/kinematics.hpp>

#include "./util.hpp"

using namespace std;

namespace tobas
{
EulerPID::EulerPID()
{
  updateGain();
}

kdl::Vector EulerPID::update(
  const kdl::Euler& cur_rpy,
  const kdl::Vector& cur_gyro,
  const kdl::Euler& tar_rpy,
  const kdl::Vector& tar_gyro,
  const double& dt)
{
  // Calculate errors
  const auto ep = computeProportionalError(cur_rpy, tar_rpy);
  const auto ed = computeDerivativeError(cur_rpy, cur_gyro, tar_gyro);

  // Accumulate integral error when using integral control.
  for (size_t i = 0; i < 3; ++i) {
    if (ki_(i) > 0.0) {
      ei_(i) += ep(i) * dt;
    }
  }

  // Calculate target Euler angle acceleration.
  const auto tar_ddrpy = kp_.hadamard(ep) + ki_.hadamard(ei_) + kd_.hadamard(ed);

  // Convert Euler angle acceleration to differential gyro.
  const auto cur_drpy = eigen::eulerrateFromAngvelLocal(cur_gyro.data, cur_rpy.roll, cur_rpy.pitch);
  return eigen::angaccFromEuleraccLocal(cur_rpy.roll, cur_rpy.pitch, cur_drpy, tar_ddrpy.data);
}

bool EulerPID::setNaturalFreq(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value < 0.0) {
    cerr << "Natural frequency must be non-negative." << endl;
    return false;
  }

  natural_freq_(idx) = value;
  updateGain();

  return true;
}

bool EulerPID::setDampingRatio(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value < 0.0) {
    cerr << "Damping ratio must be non-negative." << endl;
    return false;
  }

  damp_ratio_(idx) = value;
  updateGain();

  return true;
}

bool EulerPID::setIntegralGain(int idx, double value)
{
  if (!checkIndex(idx)) {
    return false;
  }

  if (value < 0.0) {
    cerr << "Integral gain must be non-negative." << endl;
    return false;
  }

  ki_(idx) = value;
  ei_(idx) = 0.0;

  return true;
}

void EulerPID::updateGain()
{
  kp_ = natural_freq_.sqr();
  kd_ = 2 * damp_ratio_.hadamard(natural_freq_);
}

kdl::Vector EulerPID::computeProportionalError(const kdl::Euler& cur_rpy, const kdl::Euler& tar_rpy)
{
  // Note that a straight line between two Euler angles is not the shortest rotation path.
  const auto roll_err = algo::wrapPi(tar_rpy.roll - cur_rpy.roll);
  const auto pitch_err = algo::wrapPi(tar_rpy.pitch - cur_rpy.pitch);
  const auto yaw_err = algo::wrapPi(tar_rpy.yaw - cur_rpy.yaw);
  return { roll_err, pitch_err, yaw_err };
}

kdl::Vector
EulerPID::computeDerivativeError(const kdl::Euler& cur_rpy, const kdl::Vector& cur_gyro, const kdl::Vector& tar_gyro)
{
  const auto gyro_error = tar_gyro - cur_gyro;
  return eigen::eulerrateFromAngvelLocal(gyro_error.data, cur_rpy.roll, cur_rpy.pitch);
}
}  // namespace tobas
