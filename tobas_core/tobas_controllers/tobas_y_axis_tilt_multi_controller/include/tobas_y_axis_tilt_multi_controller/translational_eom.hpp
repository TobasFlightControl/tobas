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
  static constexpr double kMinVerticalForcePerMass = 1.;  // [m/s^2]

public:
  explicit TranslationalEoM(const kdl::Tree& tree);

  bool updateInternalDataStructures();

  /**
   * @brief 並進の運動方程式を解き，目標加速度を2軸推力とロール角に変換する．
   *
   * ロール系と他の系を分離するためには現在のピッチ，ヨーを用いて計算すべきだが，
   * ジンバルロックによりヨー角が不安定になる恐れがあるため，それらの目標値を用いて計算する．
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
