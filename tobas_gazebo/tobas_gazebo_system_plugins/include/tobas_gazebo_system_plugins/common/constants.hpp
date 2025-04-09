#pragma once

#include <chrono>

namespace gazebo
{
static constexpr std::chrono::steady_clock::duration kWarnStartTime = std::chrono::seconds(3);
static constexpr double kWarnPeriod = 3.;              // [s]
static constexpr double kErrorPeriod = 1.;             // [s]
static constexpr double kRotorSpeedSlowdownSim = 10.;  // [-]
}  // namespace gazebo
