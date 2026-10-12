// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_tools/mixer_i.hpp>

#include "./mixer_solution.hpp"

namespace tobas
{
namespace y_axis_tilt_multicopter
{
/** memo: 3-39 */
class Mixer : public MixerI
{
  using super = MixerI;

public:
  explicit Mixer(const Drone& _drone, const kdl::Tree& _tree);

  void updateInternalDataStructures() override;

  std::expected<MixerSolution, std::string> solve(
    const kdl::JntArray& _cur_q,
    const kdl::Vector& _cur_gyro_b,
    const kdl::Vector& _tar_dgyro_b,
    double _ux,
    double _uz,
    const kdl::Vector& _ext_torque_b = kdl::Vector::Zero());

private:
  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  struct StaticRotorLinkInfo
  {
    bool is_tilt;
    size_t column;
    int sign;  ///< Signs of the Y components of tilt axes viewed from the body frame.
  };
  std::vector<StaticRotorLinkInfo> info_;

  struct DynamicRotorLinkState
  {
    double alpha;  ///< Tilt angle offsets viewed from the body frame [rad].
  };
  std::vector<DynamicRotorLinkState> state_;

  Eigen::Matrix5Xd E_;
  Eigen::Vector5d f_;
};
}  // namespace y_axis_tilt_multicopter
}  // namespace tobas
