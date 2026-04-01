// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_drone_tools/mixer_i.hpp>
#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_quadprog/dual_active_set.hpp>

namespace tobas
{
namespace nonplanar_multicopter
{
/**
 * @brief 制約を考慮した非平面配置マルチコプターの推力ミキシング (memo: 2-49)
 */
class QpMixer : public MixerI
{
  using super = MixerI;

public:
  explicit QpMixer(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures() override;

  bool solve(
    const kdl::JntArray& cur_q,
    const kdl::Rotation& cur_rot,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_acc_W,
    const kdl::Vector& tar_dgyro_B,
    const kdl::Vector& ext_force_W = kdl::Vector::Zero(),
    const kdl::Vector& ext_torque_B = kdl::Vector::Zero());

  const Eigen::VectorXd& getThrusts() const;
  double getThrust(size_t idx) const;

  bool setLinearWeight(double p);
  bool setAngularWeight(double p);
  bool setThrustWeight(double p);
  bool setDeltaThrustWeight(double p);

private:
  struct Config
  {
    double linear_weight = 1.;
    double angular_weight = 1.;
    double thrust_weight = 1e-9;
    double delta_thrust_weight = 1e-9;
  } cfg_;

  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  quadprog::DualActiveSetSolver qp_;  // QPソルバー
  Eigen::Diagonal6d Q_;               // EoMの重み
  Eigen::DiagonalXd R_;               // 推力の重み
  Eigen::DiagonalXd S_;               // 推力の変化量の重み
  Eigen::Matrix6Xd G_;                // EoM行列等式の左辺
  Eigen::Vector6d h_;                 // EoM行列等式の右辺
  Eigen::MatrixXd A_;                 // 不等式制約の左辺
  Eigen::VectorXd b_;                 // 不等式制約の右辺
  Eigen::VectorXd x_prev_;            // 前回の最適解

  void resizeAndFill();
};
}  // namespace nonplanar_multicopter
}  // namespace tobas
