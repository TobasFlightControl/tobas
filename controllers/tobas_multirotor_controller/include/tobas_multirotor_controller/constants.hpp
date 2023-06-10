#pragma once

#include <string>

namespace tobas_multirotor_controller
{
static constexpr double kGravity = 9.80665;  // [m/s^2]

static const std::string kCtrlName = "tobas_multirotor_controller";
static constexpr double kWarnPeriod = 1.;              // [s]
static constexpr double kCheckTopicsTimerPeriod = 5.;  // [s]
static constexpr double kInitialElevation = 1.;        // [m]
}  // namespace tobas_multirotor_controller
