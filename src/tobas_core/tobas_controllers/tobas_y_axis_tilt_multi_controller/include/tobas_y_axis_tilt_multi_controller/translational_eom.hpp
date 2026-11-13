// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/tree_mass_holder.hpp>

namespace tobas::y_axis_tilt_multicopter
{
/** memo: 3-38 */
class TranslationalEoM
{
public:
  explicit TranslationalEoM(const kdl::Tree& _tree);

  void updateInternalDataStructures();

  /**
   * Solve the translational equations of motion and convert target acceleration into two-axis thrust and roll angle.
   *
   * To separate the roll system from the other systems, the current pitch and yaw should be used,
   * but their target values are used instead because gimbal lock may make the yaw angle unstable.
   */
  bool solve(
    const kdl::Vector& _tar_acc_w,
    double _tar_pitch,
    double _tar_yaw,
    const kdl::Vector& _ext_force_w,
    double& _ux_out,
    double& _uz_out,
    kdl::Rotation& _rot_out);

private:
  kdl::TreeMassHolder mass_holder_;

  const kdl::Vector grav_W_;
};
}  // namespace tobas::y_axis_tilt_multicopter
