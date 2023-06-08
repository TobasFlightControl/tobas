#pragma once

namespace tobas_real
{
static constexpr double kGravity = 9.80665;  // [m/s^2]

static constexpr uint32_t kServoRailSize = 14;
static constexpr double kPwmFrequency = 50.;           // [Hz]
static constexpr double kPwmMin = 1000.;               // [us]
static constexpr double kPwmMax = 2000.;               // [us]
static constexpr double kPwmDisarm = 900.;             // [us]

static constexpr double kInfoPeriod = 1.;              // [s]
static constexpr double kCheckTopicsTimerPeriod = 5.;  // [s]
static constexpr double kCheckDelayThreshold = 0.02;   // [s]
}  // namespace tobas_real
