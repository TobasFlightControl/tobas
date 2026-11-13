// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/vector.hpp>

namespace tobas
{
class PositionPID
{
public:
  explicit PositionPID();

  kdl::Vector update(
    const kdl::Vector& _cur_pos,
    const kdl::Vector& _cur_vel,
    const kdl::Vector& _tar_pos,
    const kdl::Vector& _tar_vel,
    double _dt);

  void setProportionalGain(int _idx, double _value);
  void setIntegralGain(int _idx, double _value);
  void setDerivativeGain(int _idx, double _value);

  void setNaturalFreq(int _idx, double _value);
  void setDampingRatio(int _idx, double _value);
  void setMaxIntegralAccel(int _idx, double _value);

  inline const kdl::Vector& getIntegralError() const;
  inline void resetIntegralError();

private:
  // Gain
  kdl::Vector kp_ = { 1.0, 1.0, 1.0 };  ///< [/s^2]
  kdl::Vector ki_ = { 0.0, 0.0, 0.0 };  ///< [/s^3]
  kdl::Vector kd_ = { 2.0, 2.0, 2.0 };  ///< [/s]

  // Second-order form
  kdl::Vector natural_freq_ = { 1.0, 1.0, 1.0 };  ///< [rad/s]
  kdl::Vector damp_ratio_ = { 1.0, 1.0, 1.0 };    ///< [-]

  // Limit
  kdl::Vector max_i_acc_ = { INFINITY, INFINITY, INFINITY };

  // Error
  kdl::Vector ei_ = kdl::Vector::Zero();

  void setGainFromSecondOrderFrom();
};

inline const kdl::Vector& PositionPID::getIntegralError() const
{
  return ei_;
}

inline void PositionPID::resetIntegralError()
{
  ei_.setZero();
}
}  // namespace tobas
