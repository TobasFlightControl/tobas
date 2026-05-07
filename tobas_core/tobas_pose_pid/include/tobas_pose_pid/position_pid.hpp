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
    const kdl::Vector& cur_pos,
    const kdl::Vector& cur_vel,
    const kdl::Vector& tar_pos,
    const kdl::Vector& tar_vel,
    const double& dt);

  bool setProportionalGain(int idx, double value);
  bool setIntegralGain(int idx, double value);
  bool setDerivativeGain(int idx, double value);

  bool setNaturalFreq(int idx, double value);
  bool setDampingRatio(int idx, double value);
  bool setMaxIntegralAccel(int idx, double value);

  inline const kdl::Vector& getIntegralError() const;
  inline void resetIntegralError();

private:
  // Gain
  kdl::Vector kp_ = { 1., 1., 1. };  // [/s^2]
  kdl::Vector ki_ = { 0., 0., 0. };  // [/s^3]
  kdl::Vector kd_ = { 2., 2., 2. };  // [/s]

  // Second-order form
  kdl::Vector natural_freq_ = { 1., 1., 1. };  // [rad/s]
  kdl::Vector damp_ratio_ = { 1., 1., 1. };    // [-]

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
