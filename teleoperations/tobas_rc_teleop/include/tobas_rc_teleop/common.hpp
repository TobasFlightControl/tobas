#pragma once

#include <cmath>

namespace tobas_rc_teleop
{
// Constants
static constexpr double kErrorPeriod = 3.;  // [s]

// Default parameters
static constexpr double kDefaultDeadZoneRate = 0.1;
static constexpr double kDefaultMaxHorPosErr = 5.;       // [m]
static constexpr double kDefaultMaxVerPosErr = 3.;       // [m]
static constexpr double kDefaultMaxHorVel = 3.;          // [m/s]
static constexpr double kDefaultMaxVerVel = 3.;          // [m/s]
static constexpr double kDefaultMaxHorAcc = 5.;          // [m/s^2]
static constexpr double kDefaultMaxVerAcc = 4.;          // [m/s^2]
static constexpr double kDefaultMaxAttitude = M_PI / 6;  // [rad]
static constexpr double kDefaultMaxYawrate = M_PI;       // [rad/s]
static constexpr double kDefaultMaxYawErr = M_PI;        // [rad]
}  // namespace tobas_rc_teleop
