// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_quadprog/dual_active_set.hpp>
#include <tobas_tools/mixer_i.hpp>

namespace tobas
{
namespace nonplanar_multicopter
{
/** Thrust mixing for nonplanar multicopters with constraints (memo: 2-49). */
class QpMixer : public MixerI
{
  using super = MixerI;

public:
  explicit QpMixer(const Drone& _drone, const kdl::Tree& _tree);

  void updateInternalDataStructures() override;

  std::expected<Eigen::VectorXd, std::string> solve(
    const kdl::JntArray& _cur_q,
    const kdl::Rotation& _cur_rot,
    const kdl::Vector& _cur_gyro_b,
    const kdl::Vector& _tar_acc_w,
    const kdl::Vector& _tar_dgyro_b,
    const kdl::Vector& _ext_force_w = kdl::Vector::Zero(),
    const kdl::Vector& _ext_torque_b = kdl::Vector::Zero());

  void setLinearWeight(double _p);
  void setAngularWeight(double _p);
  void setThrustWeight(double _p);

private:
  struct Config
  {
    double linear_weight = 1.0;
    double angular_weight = 1.0;
    double thrust_weight = 1e-9;
  } cfg_;

  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  quadprog::DualActiveSetSolver qp_;  ///< QP solver.
  Eigen::Diagonal6d Q_;               ///< EoM weights.
  Eigen::DiagonalXd R_;               ///< Thrust weights.
  Eigen::Matrix6Xd G_;                ///< Left-hand side of the EoM matrix equality.
  Eigen::Vector6d h_;                 ///< Right-hand side of the EoM matrix equality.

  void resizeAndFill();
};
}  // namespace nonplanar_multicopter
}  // namespace tobas
