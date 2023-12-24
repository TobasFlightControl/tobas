#pragma once

#include <string>

#include <tobas_std_tools/math.hpp>

namespace tobas_fixed_wing_mpc
{
static constexpr double kStandardAirDensity = 1.225;  // [kg/m^3]

static constexpr double kWarnPeriod = 1.;                             // [s]
static constexpr double kInitialDeltaPitch = tobas_std::deg2rad(5.);  // TODO: 初期ピッチ角 (NED)

static constexpr size_t kCtrlSize = 8;
static constexpr size_t kCtrlIdx_u = 0;
static constexpr size_t kCtrlIdx_alpha = 1;
static constexpr size_t kCtrlIdx_beta = 2;
static constexpr size_t kCtrlIdx_phi = 3;
static constexpr size_t kCtrlIdx_theta = 4;
static constexpr size_t kCtrlIdx_p = 5;
static constexpr size_t kCtrlIdx_q = 6;
static constexpr size_t kCtrlIdx_r = 7;
}  // namespace tobas_fixed_wing_mpc
