// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_tools/mixer_i.hpp>

namespace tobas
{
namespace planar_multicopter
{
/* Thrust mixing for multicopters using a pseudoinverse matrix. */
class PinvMixer : public MixerI
{
  using super = MixerI;

public:
  explicit PinvMixer(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures() override;

  bool solve(
    const kdl::JntArray& cur_q,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_dgyro_B,
    const double& tar_thrusts_sum,
    const kdl::Vector& ext_torque_B = kdl::Vector::Zero());

  double getThrust(size_t idx) const;

private:
  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  Eigen::Matrix4Xd E_;
  Eigen::Vector4d f_;
  Eigen::VectorXd x_;
};
}  // namespace planar_multicopter
}  // namespace tobas
