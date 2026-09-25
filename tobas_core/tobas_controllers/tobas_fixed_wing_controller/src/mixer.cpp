// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_fixed_wing_controller/mixer.hpp"

#include <ranges>

#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_tools/fixed_wing.hpp>

namespace tobas
{
namespace fixed_wing
{
Mixer::Mixer(const Drone& drone, const kdl::Tree& tree)
  : super(drone, tree), fk_solver_(tree), inertia_solver_(tree)
{
}

bool Mixer::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  q_0_.resize(tree_.getNrOfJoints());
  q_0_.setZero();

  if (!fk_solver_.updateInternalDataStructures()) {
    return false;
  }
  if (!inertia_solver_.updateInternalDataStructures()) {
    return false;
  }

  // Compute forward kinematics.
  if (fk_solver_.jntToCart(q_0_) < 0) {
    std::cerr << "Forward kinematics failed: " << fk_solver_.errorMessage() << std::endl;
    return false;
  }

  // Compute mass properties.
  if (inertia_solver_.jntToCart(q_0_) < 0) {
    std::cerr << "Inertia solver failed: " << inertia_solver_.errorMessage() << std::endl;
    return false;
  }

  if (!calcShortPeriodModeAngularFreq()) {
    return false;
  }

  E_.conservativeResize(Eigen::NoChange, drone_.fixed_wing->numControlSurfaces());
  x_.conservativeResize(drone_.fixed_wing->numControlSurfaces());

  return true;
}

bool Mixer::solve(
  const double& dt,
  const double& rho,
  const kdl::Vector& cur_vel_B,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_dgyro_B)
{
  const auto V = cur_vel_B.norm();
  const auto& inertia = inertia_solver_.getInertia();
  const auto I_B = inertia.getRotationalInertiaCoG();

  // Left-hand side of the EoM matrix equality.
  for (const auto& [idx, pair] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto& cs = pair.second;
    E_(0, idx) = dynamicPressure(rho, std::max(V, kLowerLimitSpeed)) * drone_.fixed_wing->vehicle.wing_surface * drone_.fixed_wing->vehicle.wing_span * cs.c_roll_delta; // [Nm / rad]
    E_(1, idx) = dynamicPressure(rho, std::max(V, kLowerLimitSpeed)) * drone_.fixed_wing->vehicle.wing_surface * drone_.fixed_wing->vehicle.mac * cs.c_pitch_delta; // [Nm / rad]
  }

  // Right-hand side of the EoM matrix equality.
  kdl::Vector tar_dgyro_virtual = tar_dgyro_B; // これにより空力による応答遅れの影響を考慮
  integral_error_y_ += V / kCruiseSpeed * omega_s_  * tar_dgyro_B.y() * dt;
  tar_dgyro_virtual.y() = tar_dgyro_B.y() + integral_error_y_;
  f_ = (I_B * tar_dgyro_virtual + cur_gyro_B * (I_B * cur_gyro_B)).data.head<2>();  // [Nm]

  // Solve `Ex = f`.
  x_ = E_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f_);

  return true;
}

double Mixer::getDeflection(size_t idx) const
{
  return thrustDeadband(x_(idx));
}

bool Mixer::calcShortPeriodModeAngularFreq()
{
  // Compute mass properties.
  const auto& inertia = inertia_solver_.getInertia();
  const auto I_B = inertia.getRotationalInertiaCoG();
  const auto I_yy = I_B.iyy();

  // 短周期モードの良い近似は\ddot{\theta} - (M_q + M_alpha_dot) \dot{\theta} - M_\alpha \theta = 0
  const auto c_mac = drone_.fixed_wing->vehicle.mac;
  const auto S = drone_.fixed_wing->vehicle.wing_surface;
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
  return true;
}
}  // namespace fixed_wing
}  // namespace tobas
