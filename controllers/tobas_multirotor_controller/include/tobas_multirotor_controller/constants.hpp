#pragma once

#include <string>

namespace tobas_multirotor_controller
{
static const std::string kCtrlName = "tobas_multirotor_controller";
static const std::string kLandActionName = "multirotor_landing";
static constexpr double kWarnPeriod = 1.;               // [s]
static constexpr double kErrorPeriod = 1.;              // [s]
static constexpr double kCheckTopicsTimerPeriod = 5.;   // [s]
}  // namespace tobas_multirotor_controller
