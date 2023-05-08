#pragma once

#include <string>

namespace gazebo
{
// Constants
static constexpr double kDegreeToRadian = M_PI / 180.;

// Default values
static const std::string kDefaultWindSubTopic = "wind_speed";
static constexpr double kDefaultRotorSpeedSlowdownSim = 10.;
static constexpr double kDefaultReferenceAltitude = 500.;
}  // namespace gazebo
