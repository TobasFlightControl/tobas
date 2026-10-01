// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_fixed_wing_controller/mixer.hpp"

#include <ranges>

#include <tobas_kdl/conversion/coordinates.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_tools/fixed_wing.hpp>

namespace tobas
{
namespace fixed_wing
{
Mixer::Mixer(const Drone& drone, const kdl::Tree& tree)
  : super(drone, tree), inertia_solver_(tree)
{
}

bool Mixer::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  q_0_.resize(tree_.getNrOfJoints());
  q_0_.setZero();

  if (!inertia_solver_.updateInternalDataStructures()) {
    return false;
  }

  // Compute mass properties.
  if (inertia_solver_.jntToCart(q_0_) < 0) {
    std::cerr << "Inertia solver failed: " << inertia_solver_.errorMessage() << std::endl;
    return false;
  }

  max_roll_effectiveness_ = 0.0;
  max_pitch_effectiveness_ = 0.0;
  max_roll_effectiveness_ = 0.0;
  for (const auto& [_, cs] : drone_.fixed_wing->control_surfaces) {
    const auto& joint = tree_.getSegment(cs.link_name)->second.segment.joint();
    const auto max_deflection = joint.upper_limit;
    if (cs.type == ControlSurfaceType::kAileron) {
      max_roll_effectiveness_ += std::abs(cs.c_roll_delta) * max_deflection;
    }
    if (cs.type == ControlSurfaceType::kElevator) {
      max_pitch_effectiveness_ += std::abs(cs.c_pitch_delta) * max_deflection;
    }
    if (cs.type == ControlSurfaceType::kRudder) {
      max_yaw_effectiveness_ += std::abs(cs.c_yaw_delta) * max_deflection;
    }
  }

  if (!calcModesAngularFreq()) {
    return false;
  }

  E_.conservativeResize(Eigen::NoChange, drone_.fixed_wing->numControlSurfaces());
  x_.conservativeResize(drone_.fixed_wing->numControlSurfaces());

  return true;
}

bool Mixer::solve(
  const double& dt,
  const double& rho,
  const double& airspeed,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_dgyro_B)
{
  const auto& inertia = inertia_solver_.getInertia();
  const auto I_B = inertia.getRotationalInertiaCoG();

  // Left-hand side of the EoM matrix equality.
  for (const auto& [idx, pair] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto& cs = pair.second;
    const kdl::Vector effectiveness_frd = kdl::Vector(cs.c_roll_delta, cs.c_pitch_delta, cs.c_yaw_delta);
    kdl::Vector effectiveness_flu;
    kdl::vectorFrdToFlu(effectiveness_frd, effectiveness_flu);
    E_(0, idx) = dynamicPressure(rho, std::max(airspeed, kLowerLimitSpeed)) * drone_.fixed_wing->vehicle.wing_surface * drone_.fixed_wing->vehicle.wing_span * effectiveness_flu.x(); // [Nm / rad]
    E_(1, idx) = dynamicPressure(rho, std::max(airspeed, kLowerLimitSpeed)) * drone_.fixed_wing->vehicle.wing_surface * drone_.fixed_wing->vehicle.mac * effectiveness_flu.y(); // [Nm / rad]
    E_(2, idx) = dynamicPressure(rho, std::max(airspeed, kLowerLimitSpeed)) * drone_.fixed_wing->vehicle.wing_surface * drone_.fixed_wing->vehicle.wing_span * effectiveness_flu.z(); // [Nm / rad]
  }

  // Right-hand side of the EoM matrix equality.
  // clip, clipしないと積分誤差が蓄積して振動
  const auto max_droll  = dynamicPressure(rho, airspeed) * drone_.fixed_wing->vehicle.wing_surface * drone_.fixed_wing->vehicle.wing_span * max_roll_effectiveness_  / I_B.ixx();
  const auto max_dpitch = dynamicPressure(rho, airspeed) * drone_.fixed_wing->vehicle.wing_surface * drone_.fixed_wing->vehicle.mac * max_pitch_effectiveness_ / I_B.iyy();
  const auto max_dyaw   = dynamicPressure(rho, airspeed) * drone_.fixed_wing->vehicle.wing_surface * drone_.fixed_wing->vehicle.wing_span * max_yaw_effectiveness_ / I_B.izz();
  const auto max_dgyro_B = kdl::Vector(max_droll, max_dpitch, max_dyaw);
  const auto tar_dgyro_B_clipped = tar_dgyro_B.clamp(-max_dgyro_B, max_dgyro_B);
  // 空力による応答遅れの影響を考慮
  kdl::Vector tar_dgyro_virtual = tar_dgyro_B_clipped;
  integral_error_x_ += airspeed / kCruiseSpeed * omega_roll_ * tar_dgyro_B_clipped.x() * dt;
  integral_error_y_ += airspeed / kCruiseSpeed * omega_s_  * tar_dgyro_B_clipped.y() * dt;
  integral_error_z_ += airspeed / kCruiseSpeed * omega_dutch_roll_ * tar_dgyro_B_clipped.z() * dt;
  tar_dgyro_virtual.x() = tar_dgyro_B_clipped.x() + integral_error_x_;
  tar_dgyro_virtual.y() = tar_dgyro_B_clipped.y() + integral_error_y_;
  // FIX: yawの制御も考慮, 現状定常旋回への移行までの過渡応答のところでintegral_error_z_がたまってしまい, ラダーの効きが小さい場合に目標yawrateを出せない?
  tar_dgyro_virtual.z() = tar_dgyro_B_clipped.z() + integral_error_z_;
  f_ = (I_B * tar_dgyro_virtual + cur_gyro_B * (I_B * cur_gyro_B)).data; // [Nm]

  // Solve `Ex = f`.
  x_ = E_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f_);
  std::cout << "x_ : " << x_.transpose() << std::endl;

  return true;
}

double Mixer::getDeflection(size_t idx) const
{
  return thrustDeadband(x_(idx));
}

bool Mixer::calcModesAngularFreq()
{
  // Compute mass properties.
  const auto& inertia = inertia_solver_.getInertia();
  const auto I_B = inertia.getRotationalInertiaCoG();
  const auto I_xx = I_B.ixx();
  const auto I_yy = I_B.iyy();
  const auto I_zz = I_B.izz();

  // ロールモードの良い近似は\dot{p} - L_p p = 0
  const auto b = drone_.fixed_wing->vehicle.wing_span;
  const auto S = drone_.fixed_wing->vehicle.wing_surface;
  const auto L_p = 0.5 * st::kStandardAirDensity * std::pow(kCruiseSpeed, 2) * S * b * drone_.fixed_wing->aerodynamics.c_roll_p * b / (2.0 * kCruiseSpeed) / I_xx;
  if (L_p > 0) {
    return false;
  }
  omega_roll_ = - L_p;

  // 短周期モードの良い近似は\ddot{\theta} - (M_q + M_alpha_dot) \dot{\theta} - M_\alpha \theta = 0
  const auto c_mac = drone_.fixed_wing->vehicle.mac;
  // const auto M_q = 0.5 * st::kStandardAirDensity * std::pow(kCruiseSpeed, 2) * S * c_mac * drone_.fixed_wing->aerodynamics.c_pitch_q * c_mac / (2.0 * kCruiseSpeed) / I_yy;
  // const auto M_alpha_dot = 0.5 * st::kStandardAirDensity * std::pow(kCruiseSpeed, 2) * S * c_mac * drone_.fixed_wing->aerodynamics.c_pitch_alpha_rate * c_mac / (2.0 * kCruiseSpeed) / I_yy;
  const auto M_alpha = 0.5 * st::kStandardAirDensity * std::pow(kCruiseSpeed, 2) * S * c_mac * drone_.fixed_wing->aerodynamics.c_pitch_alpha / I_yy;
  // 共振周波数はs^2 + 2 \zeta \omega s + \omega^2 = 0なら\omega \sqrt{1 - 2 \zeta^2}
  // TODO: 1次の項も考慮したほうがいいのか考える
  const auto omega_squared = - M_alpha;
  if (omega_squared < 0) {
    return false;
  }
  omega_s_ = std::sqrt(omega_squared);

  // ダッチロールモードの粗い近似はs^2 - N_r s + N_beta = 0
  // const auto N_r = 0.5 * st::kStandardAirDensity * std::pow(kCruiseSpeed, 2) * S * b * drone_.fixed_wing->aerodynamics.c_yaw_r * b / (2.0 * kCruiseSpeed) / I_zz;
  const auto N_beta = 0.5 * st::kStandardAirDensity * std::pow(kCruiseSpeed, 2) * S * b * drone_.fixed_wing->aerodynamics.c_yaw_beta / I_zz;
  if (N_beta < 0) {
    return false;
  }
  omega_dutch_roll_ = std::sqrt(N_beta);
  return true;
}
}  // namespace fixed_wing
}  // namespace tobas
