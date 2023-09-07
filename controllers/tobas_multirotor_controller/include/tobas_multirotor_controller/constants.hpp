#pragma once

#include <string>

#include <dh_std_tools/math.hpp>

namespace tobas_multirotor_controller
{
static constexpr double kWarnPeriod = 1.;                     // [s]
static constexpr double kErrorPeriod = 1.;                    // [s]
static constexpr double kCheckTopicsTimerPeriod = 5.;         // [s]
static constexpr double kRollPitchYawrateThrustTimeout = 1.;  // [s]
static constexpr double kMaxAttitude = dh_std::deg2rad(60);   // [rad]
}  // namespace tobas_multirotor_controller
