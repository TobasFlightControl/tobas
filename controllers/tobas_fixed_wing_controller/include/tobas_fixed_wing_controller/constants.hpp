#pragma once

#include <string>

namespace tobas_fixed_wing_controller
{
static const std::string ctrlPrefix = "/tobas_fixed_wing_controller";
static constexpr double warnPeriod = 1.;              // [s]
static constexpr double checkTopicsTimerPeriod = 5.;  // [s]

static constexpr int dStateSize = 8;

static constexpr int dStateIdx_u = 0;
static constexpr int dStateIdx_alpha = 1;
static constexpr int dStateIdx_beta = 2;
static constexpr int dStateIdx_phi = 3;
static constexpr int dStateIdx_pitch = 4;
static constexpr int dStateIdx_p = 5;
static constexpr int dStateIdx_q = 6;
static constexpr int dStateIdx_r = 7;

static constexpr int ctrlIdx_beta = 0;
static constexpr int ctrlIdx_phi = 1;
static constexpr int ctrlIdx_theta = 2;
}  // namespace tobas_fixed_wing_controller
