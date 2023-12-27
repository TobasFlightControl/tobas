#pragma once

#include <string>

#include <tobas_std_tools/math.hpp>

namespace tobas_fixed_wing_lqd
{
static constexpr double kStandardAirDensity = 1.225;                  // [kg/m^3]
static constexpr double kWarnPeriod = 1.;                             // [s]
static constexpr double kInitialDeltaPitch = tobas_std::deg2rad(5.);  // TODO: 初期ピッチ角 (NED)
}  // namespace tobas_fixed_wing_lqd
