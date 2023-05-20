#pragma once

#include <string>

namespace gazebo
{
// Constants
static constexpr double kDegreeToRadian = M_PI / 180.;

// Default values
static const std::string kDefaultWindSubTopic = "wind_speed";

static constexpr double kDefaultLatitudeZero = 35.658099;    // 日本: 北緯35度39分29秒
static constexpr double kDefaultLongitudeZero = 139.741354;  // 日本: 東経139度44分28秒8759
static constexpr double kDefaultAltitudeZero = 0.;

static constexpr double kDefaultRotorSpeedSlowdownSim = 10.;
static constexpr double kDefaultCheckDelayThreshold = 0.02;  // [s]
}  // namespace gazebo
