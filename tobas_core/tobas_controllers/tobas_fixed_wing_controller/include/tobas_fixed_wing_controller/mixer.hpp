// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_tools/mixer_i.hpp>

namespace tobas
{
namespace fixed_wing
{
/* Thrust mixing for fixed wing aircrafts using a pseudoinverse matrix. */
class Mixer : public MixerI
{
  using super = MixerI;
  static constexpr double kCruiseSpeed = 15.0; // m/s, 短周期モード計算のための仮のクルーズ時対気速度
  static constexpr double kLowerLimitSpeed = 5.0; // m/s, 超低速時でも制御入力計算が発散しないようにするため, これ以下の速度でもこの対気速度であるとして舵面の影響を計算

public:
  explicit Mixer(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures() override;

  bool solve(
    const double& dt,
    const double& rho,
    const double& airspeed,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_dgyro_B);

  double getDeflection(size_t idx) const; // rad

private:
  kdl::TreeInertiaSolver inertia_solver_;

  // Fixed values.
  kdl::JntArray q_0_;
  double omega_roll_; // クルーズ時ロールモード角周波数
  double omega_s_; // クルーズ時短周期モード角周波数
  double omega_dutch_roll_; // クルーズ時ダッチロールモード角周波数
  double max_roll_effectiveness_;
  double max_pitch_effectiveness_;
  double max_yaw_effectiveness_;

  // Mutable values.
  kdl::Vector tar_dgyro_last_ = kdl::Vector::Zero();
  kdl::Vector tar_dgyro_virtual_ = kdl::Vector::Zero();
  Eigen::Matrix2Xd E_;
  Eigen::Vector2d f_; // [M_x, M_y, Mz]
  Eigen::VectorXd x_;

  bool calcModesAngularFreq();
};
}  // namespace fixed_wing
}  // namespace tobas
