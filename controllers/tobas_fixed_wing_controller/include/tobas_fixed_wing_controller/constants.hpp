#pragma once

#include <string>

namespace tobas_fixed_wing_controller
{
static const std::string ctrlName = "tobas_fixed_wing_controller";
static constexpr double warnPeriod = 1.;              // [s]
static constexpr double checkTopicsTimerPeriod = 5.;  // [s]

static constexpr int stateSize = 8;
static constexpr int stateIdx_u = 0;
static constexpr int stateIdx_alpha = 1;
static constexpr int stateIdx_beta = 2;
static constexpr int stateIdx_phi = 3;
static constexpr int stateIdx_theta = 4;
static constexpr int stateIdx_p = 5;
static constexpr int stateIdx_q = 6;
static constexpr int stateIdx_r = 7;

static constexpr int ctrlSize = 3;
static constexpr int ctrlIdx_beta = 0;
static constexpr int ctrlIdx_phi = 1;
static constexpr int ctrlIdx_theta = 2;
}  // namespace tobas_fixed_wing_controller
