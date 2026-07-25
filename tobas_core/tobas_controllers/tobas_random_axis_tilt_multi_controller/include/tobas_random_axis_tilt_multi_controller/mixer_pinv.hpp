// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_drone_tools/mixer_i.hpp>
#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>

namespace tobas
{
namespace random_axis_tilt_multicopter
{
/* Solve tilt-rotor multicopter mixing by variable transformation (memo: 3-16). */
class PinvMixer : public MixerI
{
  using super = MixerI;

  static constexpr double kMinVerticalForcePerMass = 1.0;  // [m/s^2]

public:
  explicit PinvMixer(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures() override;

  bool solve(
    const kdl::JntArray& cur_q,
    const kdl::Rotation& cur_rot,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_acc_W,
    const kdl::Vector& tar_dgyro_B,
    const kdl::Vector& ext_force_W = kdl::Vector::Zero(),
    const kdl::Vector& ext_torque_B = kdl::Vector::Zero());

  double getThrust(size_t idx) const;
  double getTiltAngle(size_t idx) const;

  bool setTiltAxisSingularDeclinationLB(double lb_rad);
  bool setTiltAxisSingularDeclinationUB(double ub_rad);

private:
  struct Config
  {
    double singular_declination_lb = 0.0;  // [rad]
    double singular_declination_ub = 0.0;  // [rad]
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
  Eigen::VectorXd x_;
};
}  // namespace random_axis_tilt_multicopter
}  // namespace tobas
