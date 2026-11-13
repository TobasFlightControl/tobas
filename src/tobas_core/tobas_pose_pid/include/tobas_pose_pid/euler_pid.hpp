// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/euler.hpp>

namespace tobas
{
class EulerPID
{
public:
  explicit EulerPID();

  kdl::Vector update(
    const kdl::Euler& _cur_rpy,
    const kdl::Vector& _cur_gyro,
    const kdl::Euler& _tar_rpy,
    const kdl::Vector& _tar_gyro,
    double _dt);

  void setNaturalFreq(int _idx, double _value);
  void setDampingRatio(int _idx, double _value);
  void setIntegralGain(int _idx, double _value);

  inline const kdl::Vector& getIntegralError() const;
  inline void resetIntegralError();

private:
  // Config
  kdl::Vector natural_freq_ = { 10.0, 10.0, 10.0 };  ///< [rad/s]
  kdl::Vector damp_ratio_ = { 1.0, 1.0, 1.0 };       ///< [-]

  // Gain
  kdl::Vector kp_;
  kdl::Vector kd_;
  kdl::Vector ki_ = { 0.0, 0.0, 0.0 };

  // Error
  kdl::Vector ei_ = kdl::Vector::Zero();

  void updateGain();

  static kdl::Vector computeProportionalError(const kdl::Euler& _cur_rpy, const kdl::Euler& _tar_rpy);
  static kdl::Vector
  computeDerivativeError(const kdl::Euler& _cur_rpy, const kdl::Vector& _cur_gyro, const kdl::Vector& _tar_gyro);
};

inline const kdl::Vector& EulerPID::getIntegralError() const
{
  return ei_;
}

inline void EulerPID::resetIntegralError()
{
  ei_.setZero();
}
}  // namespace tobas
