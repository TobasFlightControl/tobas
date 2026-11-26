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
  explicit TranslationalEoM(const Drone& _drone, const kdl::Tree& _tree);

  void updateInternalDataStructures();

  /**
   * @brief Solve the translational equations of motion and convert target acceleration into two-axis thrust and roll angle.
   *
   * To separate the roll system from the other systems, the current pitch and yaw should be used,
   * but their target values are used instead because gimbal lock may make the yaw angle unstable.
   */
  bool solve(
    const kdl::Vector& _tar_acc_W,
    const double& _tar_pitch,
    const double& _tar_yaw,
    double& _thrust_out,
    double& _thrust_angle_out,
    kdl::Euler& _angle_out);

  bool solveMulticopterDynamicsWithThrustAngleFixed(
    const kdl::Vector& _tar_acc_W,
    const double& _tar_yaw,
    const double& _tar_thrust_angle,
    double& _thrust_out,
    kdl::Euler& _angle_out);

  bool solveXZDynamics(
    const double& _tar_acc_x,
    const double& _tar_acc_z,
    const double& _cur_thrust_angle,
    const kdl::Rotation& _cur_rot,
    const kdl::Vector& _cur_vel_YW,
    double& _pitch_out,
    double& _thrust_out);

  bool solveZDynamics(
    const double& _tar_acc_z,
    const double& _cur_thrust_angle,
    const kdl::Rotation& _cur_rot,
    const kdl::Vector& _cur_vel_YW,
    double& _pitch_out,
    double& _thrust_out);

private:
  const Drone& drone_;
  kdl::TreeMassHolder mass_holder_;
  const kdl::Vector grav_W_;
  nlp::NewtonSolver2d forward_dynamics_solver_;

  Eigen::Vector2d forwardDynamics(
    const Eigen::Vector2d& _pitch_thrust,
    const Eigen::Vector2d& _tar_acc,
    const double& _thrust_angle,
    const kdl::Rotation& _cur_rot,
    const kdl::Vector& _cur_vel_YW);
  Eigen::Matrix2d dforwardDynamics(
    const Eigen::Vector2d& _pitch_thrust,
    const double& _thrust_angle,
    const kdl::Rotation& _cur_rot,
    const kdl::Vector& _cur_vel_YW);
};
}  // namespace y_axis_tilt_vtol
}  // namespace tobas
