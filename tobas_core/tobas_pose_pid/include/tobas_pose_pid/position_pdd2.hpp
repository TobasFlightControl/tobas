// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/vector.hpp>

namespace tobas
{
class PositionPDD2
{
public:
  explicit PositionPDD2();

  kdl::Vector update(
    const kdl::Vector& cur_pos,
    const kdl::Vector& cur_vel,
    const kdl::Vector& cur_acc,
    const kdl::Vector& tar_pos,
    const kdl::Vector& tar_vel,
    const kdl::Vector& tar_acc,
    const double& dt);

  bool setNaturalFreq(int idx, double value);
  bool setInertiaRatio(int idx, double value);
  bool setDampingRatio(int idx, double value);

private:
  // Config
  kdl::Vector wn_ = { 1.0, 1.0, 1.0 };    // [rad/s]
  kdl::Vector zeta_ = { 1.0, 1.0, 1.0 };  // [-]
  kdl::Vector xi_ = { 1.0, 1.0, 1.0 };    // [-]

  // Gain
  kdl::Vector kp_;
  kdl::Vector kv_;
  kdl::Vector ka_;

  kdl::Vector cmd_acc_ = kdl::Vector::Zero();

  void updateGain();
};
}  // namespace tobas
