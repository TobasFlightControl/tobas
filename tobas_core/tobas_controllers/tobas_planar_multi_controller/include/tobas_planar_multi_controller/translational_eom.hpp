// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/tree_mass_holder.hpp>

namespace tobas
{
namespace planar_multicopter
{
/* 並進の運動方程式を用いて，加速度から推力と姿勢角を求める． */
class TranslationalEoM
{
public:
  explicit TranslationalEoM(const kdl::Tree& tree);

  bool updateInternalDataStructures();

  bool solve(
    const kdl::Rotation& cur_rot,
    const kdl::Vector& tar_acc_W,
    const kdl::Vector& ext_force_W,
    double& thrust_out,
    double& roll_out,
    double& pitch_out);

private:
  kdl::TreeMassHolder mass_holder_;

  const kdl::Vector grav_W_;
  double roll_, pitch_, yaw_;
};
}  // namespace planar_multicopter
}  // namespace tobas
