#pragma once

#include <string>

namespace tobas_multirotor_controller
{
static constexpr double kGravity = 9.80665;  // [m/s^2]

static const std::string kCtrlName = "tobas_multirotor_controller";
static const std::string kLandActionName = "multirotor_land";
static constexpr double kWarnPeriod = 1.;               // [s]
static constexpr double kErrorPeriod = 1.;              // [s]
static constexpr double kCheckTopicsTimerPeriod = 5.;   // [s]
}  // namespace tobas_multirotor_controller
