// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_drone_tools/mixer_i.hpp>
#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_quadprog/dual_active_set.hpp>
#include <tobas_time_tools/stopwatch.hpp>

namespace tobas
{
namespace planar_multicopter
{
/* Thrust mixing for multicopters with constraints (memo: 3-1). */
class QpMixer : public MixerI
{
  using super = MixerI;

public:
  explicit QpMixer(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures() override;

  bool solve(
    const kdl::JntArray& cur_q,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_dgyro_B,
    const double& tar_thrusts_sum,
    const kdl::Vector& ext_torque_B = kdl::Vector::Zero());

  double getThrust(size_t idx) const;

  bool setBaseWeight(double p);
  bool setThrustWeight(double p);

private:
  struct Config
  {
    double base_weight = 1.0;
    double thrust_weight = 1e-9;
  } cfg_;

  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  quadprog::DualActiveSetSolver qp_;  // QP solver.
  Eigen::Diagonal3d Q_;               // EoM weights.
  Eigen::DiagonalXd R_;               // Thrust weights.
  Eigen::Matrix3Xd G_;                // Left-hand side of the EoM matrix equality.
  Eigen::Vector3d h_;                 // Right-hand side of the EoM matrix equality.

  tim::Stopwatch stopwatch_;

  void resizeAndFill();
};
}  // namespace planar_multicopter
}  // namespace tobas
