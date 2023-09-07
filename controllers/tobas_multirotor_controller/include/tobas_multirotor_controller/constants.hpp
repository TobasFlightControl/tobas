#pragma once

namespace tobas_multirotor_controller
{
static constexpr double kWarnPeriod = 1.;                     // [s]
static constexpr double kErrorPeriod = 1.;                    // [s]
static constexpr double kCheckTopicsTimerPeriod = 5.;         // [s]
static constexpr double kRollPitchYawrateThrustTimeout = 1.;  // [s]

static constexpr double kMaxAttitude = M_PI / 3;              // [rad]
static constexpr double kMaxHeadingError = M_PI;              // [rad]
}  // namespace tobas_multirotor_controller
