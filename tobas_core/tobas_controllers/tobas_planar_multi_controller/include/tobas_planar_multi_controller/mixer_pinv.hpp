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
  explicit PinvMixer(const Drone& drone, const kdl::Tree& tree);

  void updateInternalDataStructures() override;

  std::expected<Eigen::VectorXd, std::string> solve(
    const kdl::JntArray& cur_q,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_dgyro_B,
    double tar_thrusts_sum,
    const kdl::Vector& ext_torque_B = kdl::Vector::Zero());

private:
  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  Eigen::Matrix4Xd E_;
  Eigen::Vector4d f_;
};
}  // namespace planar_multicopter
}  // namespace tobas
