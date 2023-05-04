#pragma once

#include <string>

namespace tobas_multirotor_controller
{
static const std::string ctrlPrefix = "/tobas_multirotor_controller";
static const double warnPeriod = 1.;              // [s]
static const double checkTopicsTimerPeriod = 5.;  // [s]
static const double initialElevation = 1.;        // [m]
}  // namespace tobas_multirotor_controller
