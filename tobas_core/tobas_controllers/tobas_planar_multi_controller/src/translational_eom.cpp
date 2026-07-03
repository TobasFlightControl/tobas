// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_planar_multi_controller/translational_eom.hpp"

#include <iostream>

#include <tobas_std_tools/universal_constants.hpp>

namespace tobas
{
namespace planar_multicopter
{
TranslationalEoM::TranslationalEoM(const kdl::Tree& tree) : mass_holder_(tree), grav_W_(0, 0, -st::kGravity)
{
}

bool TranslationalEoM::updateInternalDataStructures()
{
  return mass_holder_.updateInternalDataStructures();
}

bool TranslationalEoM::solve(
  const kdl::Rotation& cur_rot,
  const kdl::Vector& tar_acc_W,
  const kdl::Vector& ext_force_W,
  double& thrust_out,
  double& roll_out,
  double& pitch_out)
{
  // Left-hand side of the translational EoM.
  const auto xyz = mass_holder_.getMass() * (tar_acc_W - grav_W_) - ext_force_W;
  const auto x = xyz.x();
  const auto y = xyz.y();
  const auto z = std::max(xyz.z(), 0.);  // Thrust cannot be generated vertically downward.

  // Compute current Euler angles.
  cur_rot.getRPY(roll_, pitch_, yaw_);

  // Cannot be realized when an attitude angle exceeds 90 degrees.
  if (std::abs(roll_) > M_PI_2 || std::abs(pitch_) > M_PI_2) {
    std::cerr << "Cannot solve translational EoM because the aircraft is upside-down." << std::endl;
    return false;
  }

  // Compute target attitude angles using the current heading to separate attitude tracking from heading tracking.
  const auto cos_yaw = std::cos(yaw_);
  const auto sin_yaw = std::sin(yaw_);
  pitch_out = std::atan2(x * cos_yaw + y * sin_yaw, z);
  roll_out = std::atan2(std::cos(pitch_out) * (x * sin_yaw - y * cos_yaw), z);

  // Compute target thrust with the current attitude to separate altitude tracking from attitude tracking.
  thrust_out = z / (std::cos(roll_) * std::cos(pitch_));

  return true;
}
}  // namespace planar_multicopter
}  // namespace tobas
