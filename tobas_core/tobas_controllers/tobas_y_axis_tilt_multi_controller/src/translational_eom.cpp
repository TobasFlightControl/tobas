// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_y_axis_tilt_multi_controller/translational_eom.hpp"

#include <iostream>

#include <tobas_math/float.hpp>
#include <tobas_std_tools/universal_constants.hpp>

namespace tobas
{
namespace y_axis_tilt_multicopter
{
TranslationalEoM::TranslationalEoM(const kdl::Tree& tree) : mass_holder_(tree), grav_W_(0, 0, -st::kGravity)
{
}

bool TranslationalEoM::updateInternalDataStructures()
{
  return mass_holder_.updateInternalDataStructures();
}

bool TranslationalEoM::solve(
  const kdl::Vector& tar_acc_W,
  const double& tar_pitch,
  const double& tar_yaw,
  const kdl::Vector& ext_force_W,
  double& ux_out,
  double& uz_out,
  kdl::Rotation& rot_out)
{
  // Compute the target force viewed from the world coordinate system.
  const auto& mass = mass_holder_.getMass();
  auto f_W = mass * (tar_acc_W - grav_W_) - ext_force_W;

  // When the acceleration magnitude is small, such as during landing,
  // the rate of change in the tilt-angle solution becomes relatively large.
  // The mixer ignores delay in tilt-angle tracking, so large tilt-angle displacement should be avoided.
  // Therefore, at minimum, ensure that thrust is generated vertically upward.
  f_W.z(std::max(f_W.z(), mass * kMinVerticalForcePerMass));

  // Convert the target force to the local coordinate system before roll.
  const auto f_L = kdl::Rotation::RPY(0.0, tar_pitch, tar_yaw).inverse(f_W);
  const auto& fx = f_L.x();
  const auto& fy = f_L.y();
  const auto& fz = f_L.z();

  // Compute the rotation angle around the third rotation axis.
  const auto cos_pitch = std::cos(tar_pitch);
  const auto sin_pitch = std::sin(tar_pitch);
  const auto den = fx * sin_pitch - fz * cos_pitch;
  if (den == 0.0) {
    std::cerr << "Free fall is commanded." << std::endl;
    return false;
  }
  const auto sin_phi = std::clamp(fy / den, -1.0, 1.0);
  const auto phi = std::asin(sin_phi);

  // Compute the third rotation matrix.
  const kdl::Vector n(cos_pitch, 0.0, sin_pitch);
  const auto rot_x = kdl::Rotation::Rot(n, phi);

  // Compute the thrust sum viewed from the body coordinate system.
  const auto u = rot_x.inverse(f_L);
  assert(math::isClose(u.y(), 0.0, 1e-3));
  ux_out = u.x();
  uz_out = u.z();

  // Compute the target attitude matrix after solving the nonlinear equation.
  rot_out = kdl::Rotation::RPY(0.0, tar_pitch, tar_yaw) * rot_x;

  return true;
}
}  // namespace y_axis_tilt_multicopter
}  // namespace tobas
