// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/blade_theory.hpp"

#include <tobas_math/core.hpp>
#include <tobas_math/definitions.hpp>
#include <tobas_std_tools/check.hpp>

namespace tobas::gui::sa::propulsion::electric
{
namespace
{
constexpr double a = M_2PI;    // 2D lift curve slope (ideal value)
constexpr double B = 0.9;      // Tip loss factor
constexpr double gamma = 8.0;  // Lock number (Ref: Balic Helicopter Aerodynamics p.66)
constexpr double C_d0 = 0.02;  // Profile drag coefficient (typical value)
}  // namespace

BladeTheory::BladeTheory(int _num_blades, double _radius, double _blade_chord, double _pitch_angle, double _air_density)
  : N_(_num_blades), R_(_radius), c_(_blade_chord), theta_(_pitch_angle), rho_(_air_density)
{
  TOBAS_CHECK(_num_blades > 0);
  TOBAS_CHECK(_radius > 0.0);
  TOBAS_CHECK(_blade_chord > 0.0);
  TOBAS_CHECK(0.0 < _pitch_angle && _pitch_angle < M_PI_2);
  TOBAS_CHECK(_air_density > 0.0);
}

double BladeTheory::motorConst() const
{
  return 4 * M_PI * rho_ * math::quar(R_) * C_T();
}

double BladeTheory::momentConst() const
{
  return R_ * lambda();
}

double BladeTheory::dragConst() const
{
  return 4 * M_PI * rho_ * math::cube(R_) * C_H();
}

double BladeTheory::sigma() const
{
  return (N_ * c_) / (M_PI * R_);
}

double BladeTheory::lambda() const
{
  const auto a_B_sigma = a * B * sigma();
  return a_B_sigma * B / 16 * (std::sqrt(1 + (64 * theta_) / (3 * a_B_sigma)) - 1);
}

double BladeTheory::C_T() const
{
  return 2 * math::sqr(lambda());
}

double BladeTheory::C_H() const
{
  const auto lam = lambda();
  const auto b0 = 0.5 * gamma * (theta_ / 4 - lam / 3);
  const auto b1c = 2 * (lam - (4.0 / 3) * theta_);  // divided by mu
  const auto b1s = -(4.0 / 3) * b0;                 // divided by mu
  return (sigma() / 4) *
         (C_d0 + (a / 6) * (2 * theta_ * (3 * lam - 2 * b1c) + 9 * lam * b1c + 2 * b0 * b1s + 3 * math::sqr(b0)));
}
}  // namespace tobas::gui::sa::propulsion::electric
