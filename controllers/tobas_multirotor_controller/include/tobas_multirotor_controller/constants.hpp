#pragma once

#include <string>

namespace tobas_multirotor_controller
{
static const std::string ctrlPrefix = "/tobas_multirotor_controller";
static constexpr double warnPeriod = 1.;              // [s]
static constexpr double checkTopicsTimerPeriod = 5.;  // [s]
static constexpr double initialElevation = 1.;        // [m]
}  // namespace tobas_multirotor_controller
