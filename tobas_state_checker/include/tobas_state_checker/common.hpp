#pragma once

namespace tobas_state_checker
{
static constexpr double kUpdateRate = 10.;                               // [Hz]
static constexpr double kWarnPeriod = 3.;                                // [s]
static constexpr double kWaitForActionServer = 3.;                       // [s]
static constexpr double kPositionStddevThreshold = 0.5;                  // [m]
static constexpr double kAttitudeStddevThreshold = dh_std::deg2rad(5.);  // [rad]
static constexpr double kHeadingStddevThreshold = dh_std::deg2rad(30.);  // [rad] // TODO
static constexpr double kBaseStateTimeout = 0.5;                         // [s]
static constexpr double kCommandTimeout = 0.5;                           // [s]
static constexpr double kAttitudeThreshold = dh_std::deg2rad(80.);       // [rad]
}  // namespace tobas_state_checker
