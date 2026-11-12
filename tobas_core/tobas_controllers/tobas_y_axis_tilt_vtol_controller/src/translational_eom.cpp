// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_y_axis_tilt_vtol_controller/translational_eom.hpp"

#include <iostream>

#include <tobas_tools/fixed_wing.hpp>
#include <tobas_math/float.hpp>
#include <tobas_math/core.hpp>
#include <tobas_std_tools/universal_constants.hpp>

#include "tobas_y_axis_tilt_vtol_controller/constants.hpp"

namespace tobas
{
namespace y_axis_tilt_vtol
{
TranslationalEoM::TranslationalEoM(const Drone& drone, const kdl::Tree& tree) : drone_(drone), mass_holder_(tree), grav_W_(0, 0, -st::kGravity)
{
}

bool TranslationalEoM::updateInternalDataStructures()
{
  if (!drone_.isValid()) {
    return false;
  }

  if (!mass_holder_.updateInternalDataStructures()) {
    return false;
  }

  return true;
}

bool TranslationalEoM::solve(
  const kdl::Vector& tar_acc_W,
  const double& tar_pitch,
  const double& tar_yaw,
  double& thrust_out,
  double& thrust_angle_out,
  kdl::Euler& angle_out)
{
  // Compute the target force viewed from the world coordinate system.
  const auto& mass = mass_holder_.getMass();
  auto f_W = mass * (tar_acc_W - grav_W_);

  // When the acceleration magnitude is small, such as during landing,
  // the rate of change in the tilt-angle solution becomes relatively large.
  // The mixer ignores delay in tilt-angle tracking, so large tilt-angle displacement should be avoided.
  // Therefore, at minimum, ensure that thrust is generated vertically upward.
  f_W.z(std::max(f_W.z(), mass * kMinVerticalForcePerMass));

  // Convert the target force to the local coordinate system before roll.
  const auto f_L = kdl::Rotation::RPY(0.0, tar_pitch, tar_yaw).inverse(f_W);
  const auto& fx = f_L.x();
  const auto& fy = f_L.y();
  const auto& fz = f_L.z();

  // Compute the rotation angle around the third rotation axis.
  const auto cos_pitch = std::cos(tar_pitch);
  const auto sin_pitch = std::sin(tar_pitch);
  const auto den = fx * sin_pitch - fz * cos_pitch;
  if (den == 0.0) {
    std::cerr << "Free fall is commanded." << std::endl;
    return false;
  }
  const auto sin_phi = std::clamp(fy / den, -1.0, 1.0);
  const auto phi = std::asin(sin_phi);

  // Compute the third rotation matrix.
  const kdl::Vector n(cos_pitch, 0.0, sin_pitch);
  const auto rot_x = kdl::Rotation::Rot(n, phi);

  // Compute the thrust sum viewed from the body coordinate system.
  const auto u = rot_x.inverse(f_L);
  assert(math::isClose(u.y(), 0.0, 1e-3));
  thrust_out = std::sqrt(math::sqr(u.x()) + math::sqr(u.z()));
  thrust_angle_out = std::atan2(u.x(), u.z());

  // Compute the target attitude after solving the nonlinear equation.
  angle_out = kdl::Euler(kdl::Rotation::RPY(0.0, tar_pitch, tar_yaw) * rot_x);

  return true;
}

bool TranslationalEoM::solveXZDynamics(
  const double& tar_acc_x,
  const double& tar_acc_z,
  const double& cur_thrust_angle,
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_vel_YW,
  double& pitch_out,
  double& thrust_out)
{
  // 目標加速度
  Eigen::Vector2d tar_acc(tar_acc_x, tar_acc_z);

  // 初期値（pitch, thrust）
  Eigen::Vector2d pitch_thrust_init(pitch_out, thrust_out);

  // NewtonSolverの初期化
  forward_dynamics_solver_.initialize(
    [this, tar_acc, cur_thrust_angle, cur_rot, cur_vel_YW]
    (const Eigen::Vector2d& pitch_thrust) -> Eigen::Vector2d
    {
      return forwardDynamics(
        pitch_thrust,
        tar_acc,
        cur_thrust_angle,
        cur_rot,
        cur_vel_YW);
    },
    [this, cur_thrust_angle, cur_rot, cur_vel_YW]
    (const Eigen::Vector2d& pitch_thrust) -> Eigen::Matrix2d
    {
      return dforwardDynamics(
        pitch_thrust,
        cur_thrust_angle,
        cur_rot,
        cur_vel_YW);
    }
  );

  // NewtonSolverを実行
  if (forward_dynamics_solver_.solve(pitch_thrust_init) < 0) {
    std::cerr << forward_dynamics_solver_.errorMessage() << std::endl;
    return false;
  }

  pitch_out = pitch_thrust_init(0);
  thrust_out = pitch_thrust_init(1);

  return true;
}

bool TranslationalEoM::solveZDynamics(
  const double& tar_acc_z,
  const double& cur_thrust_angle,
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_vel_YW,
  double& pitch_out,
  double& thrust_out)
{
  const auto [cur_roll, cur_pitch, cur_yaw] = cur_rot.getRPY();
  const auto path_angle = std::atan2(cur_vel_YW.z(), std::max(cur_vel_YW.x(), kAngleCalcThreshold));
  const auto mass = mass_holder_.getMass();
  const auto q = dynamicPressure(st::kStandardAirDensity, cur_vel_YW.norm());
  const auto C_L_sp = (mass * (tar_acc_z + st::kGravity)) / (std::cos(cur_roll) * std::cos(path_angle) * q * drone_.fixed_wing->vehicle.wing_surface);
  auto alpha_sp = (C_L_sp - drone_.fixed_wing->aerodynamics.c_lift_0) / drone_.fixed_wing->aerodynamics.c_lift_alpha;
  if (alpha_sp > drone_.fixed_wing->vehicle.alpha_limit.upper) {
    alpha_sp = drone_.fixed_wing->vehicle.alpha_limit.upper;
    const auto C_L_max = drone_.fixed_wing->aerodynamics.c_lift_0 + drone_.fixed_wing->aerodynamics.c_lift_alpha * alpha_sp;
    pitch_out = std::atan2(- std::tan(alpha_sp), std::cos(cur_roll)) - path_angle;
    thrust_out = (mass * tar_acc_z + mass * st::kGravity - q * drone_.fixed_wing->vehicle.wing_surface * C_L_max)
    / (- std::sin(pitch_out) * std::sin(cur_thrust_angle) + std::cos(pitch_out) * std::cos(cur_roll) * std::cos(cur_thrust_angle));
  } else {
    pitch_out = std::atan2(- std::tan(alpha_sp), std::cos(cur_roll)) - path_angle;
    thrust_out = 0.0;
  }
  return true;
}

Eigen::Vector2d TranslationalEoM::forwardDynamics(
  const Eigen::Vector2d& pitch_thrust,
  const Eigen::Vector2d& tar_acc,
  const double& thrust_angle,
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_vel_YW)
{
  // aerodynamic forces
  const auto path_angle = std::atan2(cur_vel_YW.z(), std::max(cur_vel_YW.x(), kAngleCalcThreshold));
  const auto [cur_roll, cur_pitch, cur_yaw] = cur_rot.getRPY();
  const auto pitch = pitch_thrust(0);
  const auto alpha = - std::atan(std::tan(path_angle + pitch) * std::cos(cur_roll));
  const auto& aerodynamics = drone_.fixed_wing->aerodynamics;
  const auto C_L = aerodynamics.c_lift_0 + aerodynamics.c_lift_alpha * alpha;
  const auto C_D = aerodynamics.c_drag_0 + aerodynamics.c_drag_alpha * alpha + aerodynamics.c_drag_alpha2 * math::sqr(alpha);
  const auto q = dynamicPressure(st::kStandardAirDensity, cur_vel_YW.norm());
  const auto L = q * drone_.fixed_wing->vehicle.wing_surface * C_L;
  const auto D = q * drone_.fixed_wing->vehicle.wing_surface * C_D;
  const kdl::Vector f_aero(L * std::sin(alpha) - D * std::cos(alpha), 0, L * std::cos(alpha) + D * std::sin(alpha));
  // thrust
  const auto thrust = pitch_thrust(1);
  const kdl::Vector f_thrust(thrust * std::sin(thrust_angle), 0, thrust * std::cos(thrust_angle));
  // resulting force error
  const auto& mass = mass_holder_.getMass();
  const auto rhs = kdl::Rotation::RotY(pitch) * kdl::Rotation::RotX(cur_roll) * (f_thrust + f_aero) + mass * grav_W_;
  Eigen::Matrix<double, 2, 3> C;
  C << 1.0, 0.0, 0.0,
       0.0, 0.0, 1.0;
  const auto f = mass * tar_acc - C * rhs.data;
  return f;
}


Eigen::Matrix2d TranslationalEoM::dforwardDynamics(
  const Eigen::Vector2d& pitch_thrust,
  const double& thrust_angle,
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_vel_YW)
{
  // aerodynamic forces
  const auto path_angle = std::atan2(cur_vel_YW.z(), std::max(cur_vel_YW.x(), kAngleCalcThreshold));
  const auto [cur_roll, cur_pitch, cur_yaw] = cur_rot.getRPY();
  const auto pitch = pitch_thrust(0);
  const auto alpha = - std::atan(std::tan(path_angle + pitch) * std::cos(cur_roll));
  const auto& aerodynamics = drone_.fixed_wing->aerodynamics;
  const auto C_L = aerodynamics.c_lift_0 + aerodynamics.c_lift_alpha * alpha;
  const auto C_D = aerodynamics.c_drag_0 + aerodynamics.c_drag_alpha * alpha + aerodynamics.c_drag_alpha2 * math::sqr(alpha);
  const auto q = dynamicPressure(st::kStandardAirDensity, cur_vel_YW.norm());
  const auto L = q * drone_.fixed_wing->vehicle.wing_surface * C_L;
  const auto D = q * drone_.fixed_wing->vehicle.wing_surface * C_D;
  const kdl::Vector f_aero(L * std::sin(alpha) - D * std::cos(alpha), 0, L * std::cos(alpha) + D * std::sin(alpha));
  // thrust
  const auto thrust = pitch_thrust(1);
  const kdl::Vector f_thrust(thrust * std::sin(thrust_angle), 0, thrust * std::cos(thrust_angle));
  // differentiation
  const kdl::Vector df_aero_dalpha =  q * drone_.fixed_wing->vehicle.wing_surface * kdl::Vector(
    aerodynamics.c_lift_alpha * std::sin(alpha) + C_L * std::cos(alpha) - (aerodynamics.c_drag_alpha + 2 * aerodynamics.c_drag_alpha2 * alpha) * std::cos(alpha) + C_D * std::sin(alpha),
    0,
    aerodynamics.c_lift_alpha * std::cos(alpha) - C_L * std::sin(alpha) + (aerodynamics.c_drag_alpha + 2 * aerodynamics.c_drag_alpha2 * alpha) * std::sin(alpha) + C_D * std::cos(alpha)
  );

  Eigen::Matrix<double, 2, 3> C;
  C << 1.0, 0.0, 0.0,
       0.0, 0.0, 1.0;
  Eigen::Matrix3d dR_Y_dpitch;
  dR_Y_dpitch <<
    - std::sin(pitch), 0, std::cos(pitch),
                    0, 1,               0,
    - std::cos(pitch), 0, - std::sin(pitch);
  const auto dalpha_dpitch = - math::sqr(std::cos(alpha)) / math::sqr(std::cos(path_angle + pitch)) * std::cos(cur_roll);
  const Eigen::Vector2d dfdpitch = C * (- dR_Y_dpitch * (kdl::Rotation::RotX(cur_roll) * (f_thrust + f_aero)).data
  - (kdl::Rotation::RotY(pitch) * kdl::Rotation::RotX(cur_roll) * df_aero_dalpha * dalpha_dpitch).data);
  const Eigen::Vector2d dfdthrust = C * - (kdl::Rotation::RotY(pitch) * kdl::Rotation::RotX(cur_roll) * kdl::Vector(std::sin(thrust_angle), 0, std::cos(thrust_angle))).data;
  Eigen::Matrix2d dfdx;
  dfdx.col(0) = dfdpitch;
  dfdx.col(1) = dfdthrust;
  return dfdx;
}

}  // namespace y_axis_tilt_vtol
}  // namespace tobas
