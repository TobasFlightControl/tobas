#pragma once

#include <cmath>

namespace tobas
{
namespace st
{
static constexpr double kGravity = 9.80665;           // [m/s^2] 重力加速度
static constexpr double kStandardAirDensity = 1.225;  // [kg/m^3] 標準大気における大気密度
static constexpr double kGeomagScale = 0.5;           // [G] 地磁気の磁束密度の大きさ

static constexpr double kDeg2Rad = M_PI / 180;  // deg -> rad
static constexpr double kRad2Deg = 180 / M_PI;  // rad -> deg

static constexpr double kRpmToRps = M_PI / 30.;  // RPM -> rad/s
static constexpr double kRpsToRpm = 30. / M_PI;  // rad/s -> RPM

static constexpr double kFeetToMeter = 0.3048;
static constexpr double kMeterToFeet = 1 / kFeetToMeter;
}  // namespace st
}  // namespace tobas
