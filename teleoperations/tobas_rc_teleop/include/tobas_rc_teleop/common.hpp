#pragma once

#include <cmath>

namespace tobas_rc_teleop
{
static constexpr double kDefaultMaxHorizontalVelocity = 3.;  // [m/s]
static constexpr double kDefaultMaxVerticalVelocity = 3.;    // [m/s]
static constexpr double kDefaultMaxAcceleration = 3.;        // [m/s^2]
static constexpr double kDefaultMinAcceleration = -3.;       // [m/s^2]
static constexpr double kDefaultMaxAttitude = M_PI / 6;      // [rad]
static constexpr double kDefaultMaxYawrate = M_PI;           // [rad/s]
}  // namespace tobas_rc_teleop
