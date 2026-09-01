// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_drone_core/propulsion_system/ice_propulsion_system/aerodynamics.hpp>
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
namespace ice
{
/* Approximate variable-pitch propeller aerodynamic parameters with blade theory and Taylor expansion (memo: 3-36). */
class BladeTheory
{
public:
  explicit BladeTheory(
    int num_blades,
    double radius,
    double blade_chord,
    double pitch_angle,
    double air_density = st::kStandardAirDensity);

  VppMotorConstant motorConst() const;
  VppMomentConstant momentConst() const;
  VppDragConstant dragConst() const;

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

  /* dlam / dtheta */
  double lambdaDeriv() const;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
