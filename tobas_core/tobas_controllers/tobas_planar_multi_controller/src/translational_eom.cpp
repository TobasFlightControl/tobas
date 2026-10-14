// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_planar_multi_controller/translational_eom.hpp"

#include <tobas_std_tools/universal_constants.hpp>

namespace tobas::planar_multicopter
{
TranslationalEoM::TranslationalEoM(const kdl::Tree& _tree) : mass_holder_(_tree), grav_W_(0, 0, -st::kGravity)
{
}

void TranslationalEoM::updateInternalDataStructures()
{
  mass_holder_.updateInternalDataStructures();
}

bool TranslationalEoM::solve(
  const kdl::Rotation& _cur_rot,
  const kdl::Vector& _tar_acc_w,
  const kdl::Vector& _ext_force_w,
  double& _thrust_out,
  double& _roll_out,
  double& _pitch_out)
{
  // Left-hand side of the translational EoM.
  const auto xyz = mass_holder_.getMass() * (_tar_acc_w - grav_W_) - _ext_force_w;
  const auto x = xyz.x();
  const auto y = xyz.y();
  const auto z = std::max(xyz.z(), 0.0);  // Thrust cannot be generated vertically downward.

  // Compute current Euler angles.
  _cur_rot.getRPY(roll_, pitch_, yaw_);

  // Cannot be realized when an attitude angle exceeds 90 degrees.
  if (std::abs(roll_) > M_PI_2 || std::abs(pitch_) > M_PI_2) {
    return false;  // Cannot solve translational EoM because the aircraft is upside-down.
  }

  // Compute target attitude angles using the current heading to separate attitude tracking from heading tracking.
  const auto cos_yaw = std::cos(yaw_);
  const auto sin_yaw = std::sin(yaw_);
  _pitch_out = std::atan2(x * cos_yaw + y * sin_yaw, z);
  _roll_out = std::atan2(std::cos(_pitch_out) * (x * sin_yaw - y * cos_yaw), z);

  // Compute target thrust with the current attitude to separate altitude tracking from attitude tracking.
  _thrust_out = z / (std::cos(roll_) * std::cos(pitch_));

  return true;
}
}  // namespace tobas::planar_multicopter
