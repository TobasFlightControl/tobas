#pragma once

#include <chrono>

namespace gazebo
{
static constexpr std::chrono::steady_clock::duration kWarnStartTime = std::chrono::seconds(3);
static constexpr double kWarnPeriod = 3.;              // [s]
static constexpr double kErrorPeriod = 1.;             // [s]
static constexpr double kRotorSpeedSlowdownSim = 10.;  // [-]

static constexpr double kDefaultLatitudeZero = 35.658099;    // [deg] 日本: 北緯35度39分29秒
static constexpr double kDefaultLongitudeZero = 139.741354;  // [deg] 日本: 東経139度44分28秒8759
static constexpr double kDefaultAltitudeZero = 0.;           // [m]
}  // namespace gazebo
