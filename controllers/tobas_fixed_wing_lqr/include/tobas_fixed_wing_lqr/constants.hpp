#pragma once

#include <string>

#include <dh_std_tools/math.hpp>

namespace tobas_fixed_wing_lqr
{
static constexpr double kGravity = 9.80665;                        // [m/s^2]
static constexpr double kStandardAirDensity = 1.225;               // [kg/m^3]
static const std::string kCtrlName = "tobas_fixed_wing_lqr";
static constexpr double kWarnPeriod = 1.;                          // [s]
static constexpr double kCheckTopicsTimerPeriod = 5.;              // [s]
static constexpr double kInitialDeltaPitch = dh_std::deg2rad(5.);  // TODO: 初期ピッチ角 (NED)
}  // namespace tobas_fixed_wing_lqr
