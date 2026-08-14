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
/**
 * @brief Thrust mixing for nonplanar multicopters with constraints (memo: 2-49).
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

private:
  struct Config
  {
    double linear_weight = 1.0;
    double angular_weight = 1.0;
    double thrust_weight = 1e-9;
  } cfg_;

  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  quadprog::DualActiveSetSolver qp_;  // QP solver.
  Eigen::Diagonal6d Q_;               // EoM weights.
  Eigen::DiagonalXd R_;               // Thrust weights.
  Eigen::Matrix6Xd G_;                // Left-hand side of the EoM matrix equality.
  Eigen::Vector6d h_;                 // Right-hand side of the EoM matrix equality.

  void resizeAndFill();
};
}  // namespace nonplanar_multicopter
}  // namespace tobas
