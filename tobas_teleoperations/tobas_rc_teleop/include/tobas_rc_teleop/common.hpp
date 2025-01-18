#pragma once

#include <cmath>

namespace tobas_rc_teleop
{
// Constants
static constexpr double kDeadZone = 0.05;
static constexpr double kMaxPositionError = 5.;  // [m]
static constexpr double kMaxYawError = M_PI;     // [rad]

// Default parameters
static constexpr double kDefaultMaxHorVel = 3.;        // [m/s]
static constexpr double kDefaultMaxVerVel = 3.;        // [m/s]
static constexpr double kDefaultMaxHorAcc = 5.;        // [m/s^2]
static constexpr double kDefaultMaxVerAcc = 4.;        // [m/s^2]
static constexpr double kDefaultMaxAttitude = M_PI_2;  // [rad]
static constexpr double kDefaultMaxYawrate = M_PI_2;   // [rad/s]
}  // namespace tobas_rc_teleop
