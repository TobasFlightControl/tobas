// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/tree_mass_holder.hpp>

namespace tobas
{
namespace planar_multicopter
{
/** Compute thrust and attitude angles from acceleration using the translational equations of motion. */
class TranslationalEoM
{
public:
  explicit TranslationalEoM(const kdl::Tree& _tree);

  void updateInternalDataStructures();

  bool solve(
    const kdl::Rotation& _cur_rot,
    const kdl::Vector& _tar_acc_w,
    const kdl::Vector& _ext_force_w,
    double& _thrust_out,
    double& _roll_out,
    double& _pitch_out);

private:
  kdl::TreeMassHolder mass_holder_;

  const kdl::Vector grav_W_;
  double roll_, pitch_, yaw_;
};
}  // namespace planar_multicopter
}  // namespace tobas
