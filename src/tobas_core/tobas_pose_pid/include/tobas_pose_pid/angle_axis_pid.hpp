// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/rotation.hpp>

namespace tobas
{
class AngleAxisPID
{
public:
  explicit AngleAxisPID();

  kdl::Vector update(
    const kdl::Rotation& _cur_rot,
    const kdl::Vector& _cur_gyro,
    const kdl::Rotation& _tar_rot,
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
};

inline const kdl::Vector& AngleAxisPID::getIntegralError() const
{
  return ei_;
}

inline void AngleAxisPID::resetIntegralError()
{
  ei_.setZero();
}
}  // namespace tobas
