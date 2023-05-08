#pragma once

#include <string>

namespace tobas_fixed_wing_controller
{
static constexpr double gravity = 9.80665;  // [m/s^2]
static const std::string ctrlName = "tobas_fixed_wing_controller";
static constexpr double warnPeriod = 1.;              // [s]
static constexpr double checkTopicsTimerPeriod = 5.;  // [s]

static constexpr int ctrlSize = 3;
static constexpr int ctrlIdx_beta = 0;
static constexpr int ctrlIdx_phi = 1;
static constexpr int ctrlIdx_theta = 2;
}  // namespace tobas_fixed_wing_controller
