#pragma once

#include <cmath>

namespace tobas_fixed_wing_lqd
{
static constexpr double kStandardAirDensity = 1.225;     // [kg/m^3]
static constexpr double kWarnPeriod = 1.;                // [s]
static constexpr double kInitialDeltaPitch = M_PI / 12;  // [rad]
}  // namespace tobas_fixed_wing_lqd
