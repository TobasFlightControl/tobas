#pragma once

#include <cmath>

namespace tobas_rc_teleop
{
// Constants
static constexpr double kErrorPeriod = 3.;  // [s]

// Default parameters
static constexpr double kDefaultDeadZoneRate = 0.1;
static constexpr double kDefaultMaxHorizontalVelocity = 3.;  // [m/s]
static constexpr double kDefaultMaxVerticalVelocity = 3.;    // [m/s]
static constexpr double kDefaultMaxHorizontalAccel = 5.;     // [m/s^2]
static constexpr double kDefaultMaxVerticalAccel = 4.;       // [m/s^2]
static constexpr double kDefaultMaxAttitude = M_PI / 6;      // [rad]
static constexpr double kDefaultMaxYawrate = M_PI;           // [rad/s]
}  // namespace tobas_rc_teleop
