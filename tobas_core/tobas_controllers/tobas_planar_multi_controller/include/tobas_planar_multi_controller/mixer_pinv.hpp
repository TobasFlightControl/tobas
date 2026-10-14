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
/** Thrust mixing for multicopters using a pseudoinverse matrix. */
class PinvMixer : public MixerI
{
  using super = MixerI;

public:
  explicit PinvMixer(const Drone& _drone, const kdl::Tree& _tree);

  void updateInternalDataStructures() override;

  std::expected<Eigen::VectorXd, std::string> solve(
    const kdl::JntArray& _cur_q,
    const kdl::Vector& _cur_gyro_b,
    const kdl::Vector& _tar_dgyro_b,
    double _tar_thrusts_sum,
    const kdl::Vector& _ext_torque_b = kdl::Vector::Zero());

private:
  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  Eigen::Matrix4Xd E_;
  Eigen::Vector4d f_;
};
}  // namespace planar_multicopter
}  // namespace tobas
