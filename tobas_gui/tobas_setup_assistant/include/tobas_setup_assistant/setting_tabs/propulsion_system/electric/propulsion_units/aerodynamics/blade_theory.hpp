// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_math/definitions.hpp>
#include <tobas_std_tools/universal_constants.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
/* Unsteady Aerodynamic Parameter Estimation for Multirotor Helicopters [Nguyen+, 2019] */
class BladeTheory
{
public:
  explicit BladeTheory(
    int num_blades,
    double radius,
    double blade_chord,
    double pitch_angle,
    double air_density = st::kStandardAirDensity);

  double motorConst() const;
  double momentConst() const;
  double dragConst() const;

private:
  const int N_;
  const double R_;
  const double c_;
  const double theta_;
  const double rho_;

  /* Solidity */
  double sigma() const;

  /* Inflow ratio */
  double lambda() const;

  /* Thrust coefficient */
  double C_T() const;

  /* Horizontal force coefficient (divided by mu) */
  double C_H() const;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
