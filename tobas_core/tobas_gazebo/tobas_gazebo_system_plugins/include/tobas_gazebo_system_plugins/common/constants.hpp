#pragma once

#include <chrono>

namespace gazebo
{
static constexpr auto kCheckTopicWarnStartTime = std::chrono::seconds(3);
static constexpr double kWarnPeriod = 3.;              // [s]
static constexpr double kErrorPeriod = 1.;             // [s]
static constexpr double kRotorSpeedSlowdownSim = 30.;  // [-]
}  // namespace gazebo
