#pragma once

#include <string>

namespace gazebo
{
// Constants
static constexpr double deg2rad = M_PI / 180.;

// Default values
static const std::string kDefaultWindSubTopic = "wind_speed";
static constexpr double kDefaultRotorSpeedSlowdownSim = 10.;
static constexpr double kDefaultReferenceAltitude = 500.;
}  // namespace gazebo
