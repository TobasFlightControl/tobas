// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_tools/mixer_i.hpp>

#include "./mixer_solution.hpp"

namespace tobas::random_axis_tilt_multicopter
{
/** Solve tilt-rotor multicopter mixing by variable transformation (memo: 3-16). */
class PinvMixer : public MixerI
{
  using super = MixerI;

public:
  explicit PinvMixer(const Drone& _drone, const kdl::Tree& _tree);

  void updateInternalDataStructures() override;

  std::expected<MixerSolution, std::string> solve(
    const kdl::JntArray& _cur_q,
    const kdl::Rotation& _cur_rot,
    const kdl::Vector& _cur_gyro_b,
    const kdl::Vector& _tar_acc_w,
    const kdl::Vector& _tar_dgyro_b,
    const kdl::Vector& _ext_force_w = kdl::Vector::Zero(),
    const kdl::Vector& _ext_torque_b = kdl::Vector::Zero());

  void setTiltAxisSingularDeclinationLB(double _lb_rad);
  void setTiltAxisSingularDeclinationUB(double _ub_rad);

private:
  struct Config
  {
    double singular_declination_lb = 0.0;  ///< [rad]
    double singular_declination_ub = 0.0;  ///< [rad]
  } cfg_;

  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  struct StaticRotorLinkInfo
  {
    bool is_tilt;
    Eigen::Matrix<double, 3, 2> A;
  };
  std::vector<StaticRotorLinkInfo> info_;

  struct DynamicRotorLinkState
  {
    bool is_singular = false;
  };
  std::vector<DynamicRotorLinkState> state_;

  Eigen::Matrix6Xd E_;
  Eigen::Vector6d f_;
};
}  // namespace tobas::random_axis_tilt_multicopter
