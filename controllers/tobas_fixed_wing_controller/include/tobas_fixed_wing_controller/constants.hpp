#pragma once

#include <string>

namespace tobas_fixed_wing_controller
{
static constexpr double kGravity = 9.80665;  // [m/s^2]
static const std::string kCtrlName = "tobas_fixed_wing_controller";
static constexpr double kWarnPeriod = 1.;              // [s]
static constexpr double kCheckTopicsTimerPeriod = 5.;  // [s]

static constexpr int kCtrlSize = 3;
static constexpr int kCtrlIdx_beta = 0;
static constexpr int kCtrlIdx_phi = 1;
static constexpr int kCtrlIdx_theta = 2;
}  // namespace tobas_fixed_wing_controller
