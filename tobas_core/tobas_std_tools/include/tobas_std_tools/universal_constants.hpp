#pragma once

#include <cmath>

namespace tobas_std
{
static constexpr double kGravity = 9.80665;           // [m/s^2]
static constexpr double kStandardAirDensity = 1.225;  // [kg/m^3]

static constexpr double kDeg2Rad = M_PI / 180;
static constexpr double kRad2Deg = 180 / M_PI;

static constexpr double kFeetToMeter = 0.3048;
static constexpr double kMeterToFeet = 1 / kFeetToMeter;
}  // namespace tobas_std
