// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/rotation.hpp>

namespace tobas
{
class AngleAxisPDD2
{
public:
  explicit AngleAxisPDD2();

  kdl::Vector update(
    const kdl::Rotation& _cur_rot,
    const kdl::Vector& _cur_gyro,
    const kdl::Vector& _cur_dgyro,
    const kdl::Rotation& _tar_rot,
    const kdl::Vector& _tar_gyro,
    const kdl::Vector& _tar_dgyro,
    double _dt);

  void setNaturalFreq(int _idx, double _value);
  void setInertiaRatio(int _idx, double _value);
  void setDampingRatio(int _idx, double _value);

private:
  // Config
  kdl::Vector wn_ = { 10.0, 10.0, 10.0 };  ///< [rad/s]
  kdl::Vector zeta_ = { 1.0, 1.0, 1.0 };   ///< [-]
  kdl::Vector xi_ = { 1.0, 1.0, 1.0 };     ///< [-]

  // Gain
  kdl::Vector kp_;
  kdl::Vector kv_;
  kdl::Vector ka_;

  kdl::Vector cmd_dgyro_ = kdl::Vector::Zero();

  void updateGain();
};
}  // namespace tobas
