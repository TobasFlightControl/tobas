// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/tree_mass_holder.hpp>

namespace tobas
{
namespace y_axis_tilt_multicopter
{
/* memo: 3-38 */
class TranslationalEoM
{
  static constexpr double kMinVerticalForcePerMass = 1.0;  // [m/s^2]

public:
  explicit TranslationalEoM(const kdl::Tree& tree);

  bool updateInternalDataStructures();

  /**
   * @brief Solve the translational equations of motion and convert target acceleration into two-axis thrust and roll angle.
   *
   * To separate the roll system from the other systems, the current pitch and yaw should be used,
   * but their target values are used instead because gimbal lock may make the yaw angle unstable.
   */
  bool solve(
    const kdl::Vector& tar_acc_W,
    const double& tar_pitch,
    const double& tar_yaw,
    const kdl::Vector& ext_force_W,
    double& ux_out,
    double& uz_out,
    kdl::Rotation& rot_out);

private:
  kdl::TreeMassHolder mass_holder_;

  const kdl::Vector grav_W_;
};
}  // namespace y_axis_tilt_multicopter
}  // namespace tobas
