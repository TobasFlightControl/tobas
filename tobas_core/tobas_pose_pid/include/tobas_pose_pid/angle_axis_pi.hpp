// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/rotation.hpp>

namespace tobas
{
class AngleAxisPI
{
public:
  explicit AngleAxisPI();

  kdl::Vector update(const kdl::Rotation& _cur_rot, const kdl::Rotation& _tar_rot, double _dt);

  void setProportionalGain(int _idx, double _value);
  void setIntegralGain(int _idx, double _value);

  inline const kdl::Vector& getIntegralError() const;
  inline void resetIntegralError();

private:
  // Gain
  kdl::Vector kp_ = { 0.0, 0.0, 0.0 };
  kdl::Vector ki_ = { 0.0, 0.0, 0.0 };

  // Error
  kdl::Vector ei_ = kdl::Vector::Zero();
};

inline const kdl::Vector& AngleAxisPI::getIntegralError() const
{
  return ei_;
}

inline void AngleAxisPI::resetIntegralError()
{
  ei_.setZero();
}
}  // namespace tobas
