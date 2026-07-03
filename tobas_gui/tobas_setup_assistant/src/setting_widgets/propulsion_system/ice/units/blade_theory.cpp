// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/blade_theory.hpp"

#include <tobas_math/core.hpp>
#include <tobas_std_tools/check.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
BladeTheory::BladeTheory(int num_blades, double radius, double blade_chord, double pitch_angle, double air_density)
  : N_(num_blades), R_(radius), c_(blade_chord), theta_(pitch_angle), rho_(air_density)
{
  TOBAS_CHECK(num_blades > 0);
  TOBAS_CHECK(radius > 0.);
  TOBAS_CHECK(blade_chord > 0.);
  TOBAS_CHECK(0. < pitch_angle && pitch_angle < M_PI_2);
  TOBAS_CHECK(air_density > 0.);
}

VppMotorConstant BladeTheory::motorConst() const
{
  const auto lam = lambda();
  const auto dlam = lambdaDeriv();

  const auto ct = 2 * math::sqr(lam);
  const auto dct = 4 * lam * dlam;

  const auto scale = 4 * M_PI * rho_ * math::quar(R_);
  return VppMotorConstant(scale * ct, scale * dct);
}

VppMomentConstant BladeTheory::momentConst() const
{
  const auto lam = lambda();
  const auto dlam = lambdaDeriv();
  return VppMomentConstant(R_ * dlam, R_ * lam, 0., 0.);  // FIXME: Add the 1/phi term.
}

VppDragConstant BladeTheory::dragConst() const
{
  const auto sig = sigma();
  const auto lam = lambda();
  const auto dlam = lambdaDeriv();

  const auto b0 = 0.5 * gamma * (theta_ / 4 - lam / 3);
  const auto b1c = 2 * (lam - (4. / 3) * theta_);  // divided by mu
  const auto b1s = -(4. / 3) * b0;                 // divided by mu
  const auto ch =
    (sig / 4) *
    (C_d0 + (a / 6) * (2 * theta_ * (3 * lam - 2 * b1c) + 9 * lam * b1c + 2 * b0 * b1s + 3 * math::sqr(b0)));

  const auto db0 = (gamma / 2) * (1. / 4 - dlam / 3);
  const auto db1c = 2 * (dlam - 4. / 3);
  const auto db1s = -(4. / 3) * db0;
  const auto dch = (a * sig / 24) * ((6 * lam - 4 * b1c) + (3 * theta_ + 9 * b1c) * dlam + (6 * b0 + 2 * b1s) * db0 +
                                     (9 * lam - 2 * theta_) * db1c + 2 * b0 * db1s);

  const auto scale = 4 * M_PI * rho_ * math::cube(R_);
  return VppDragConstant(scale * ch, scale * dch);
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

double BladeTheory::lambdaDeriv() const
{
  const auto a_B_sigma = a * B * sigma();
  return (2 * B / 3) / std::sqrt(1 + (64 * theta_) / (3 * a_B_sigma));
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
