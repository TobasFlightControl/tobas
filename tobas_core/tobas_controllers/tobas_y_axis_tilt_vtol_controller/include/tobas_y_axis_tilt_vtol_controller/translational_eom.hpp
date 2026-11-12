// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_drone_core/drone.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_nlp/newton_2d.hpp>

namespace tobas
{
namespace y_axis_tilt_vtol
{
class TranslationalEoM
{
  static constexpr double kMinVerticalForcePerMass = 1.0;  // [m/s^2]

public:
  explicit TranslationalEoM(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures();

  /**
   * @brief Solve the translational equations of motion and convert target acceleration into two-axis thrust and roll angle.
   *
   * To separate the roll system from the other systems, the current pitch and yaw should be used,
   * but their target values are used instead because gimbal lock may make the yaw angle unstable.
   */
  bool solve(
    const kdl::Vector& tar_acc_W,
    const double& tar_pitch,
    const double& tar_yaw,
    double& thrust_out,
    double& thrust_angle_out,
    kdl::Euler& angle_out);

  bool solveXZDynamics(
    const double& tar_acc_x,
    const double& tar_acc_z,
    const double& cur_thrust_angle,
    const kdl::Rotation& cur_rot,
    const kdl::Vector& cur_vel_YW,
    double& pitch_out,
    double& thrust_out);

  bool solveZDynamics(
    const double& tar_acc_z,
    const double& cur_thrust_angle,
    const kdl::Rotation& cur_rot,
    const kdl::Vector& cur_vel_YW,
    double& pitch_out,
    double& thrust_out);

private:
  const Drone& drone_;
  kdl::TreeMassHolder mass_holder_;
  const kdl::Vector grav_W_;
  nlp::NewtonSolver2d forward_dynamics_solver_;

  Eigen::Vector2d forwardDynamics(
    const Eigen::Vector2d& pitch_thrust,
    const Eigen::Vector2d& tar_acc,
    const double& thrust_angle,
    const kdl::Rotation& cur_rot,
    const kdl::Vector& cur_vel_YW);
  Eigen::Matrix2d dforwardDynamics(
    const Eigen::Vector2d& pitch_thrust,
    const double& thrust_angle,
    const kdl::Rotation& cur_rot,
    const kdl::Vector& cur_vel_YW);
};
}  // namespace y_axis_tilt_vtol
}  // namespace tobas
