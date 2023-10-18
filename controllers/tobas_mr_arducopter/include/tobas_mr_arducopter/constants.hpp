#pragma once

#include <cinttypes>
#include <cmath>

namespace tobas_mr_arducopter
{
// Constants
static constexpr double kRadians = M_PI / 180;
static constexpr double kDegrees = 180 / M_PI;

// default angle P gain for roll, pitch and yaw
static constexpr double ATTITUDE_CONTROL_ANGLE_P = 4.5;
// minimum body-frame acceleration limit for the stability controller (for roll and pitch axis)
static constexpr double ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS = 40. * kRadians;
// maximum body-frame acceleration limit for the stability controller (for roll and pitch axis)
static constexpr double ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS = 720. * kRadians;
// minimum body-frame acceleration limit for the stability controller (for yaw axis)
static constexpr double ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS = 10. * kRadians;
// maximum body-frame acceleration limit for the stability controller (for yaw axis)
static constexpr double ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS = 120. * kRadians;
// constraint on yaw angle error in degrees.  This should lead to maximum turn rate of
// 10deg/sec * Stab Rate P so by default will be 45deg/sec.
static constexpr double ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS = 6000;
// default maximum acceleration for roll/pitch axis in centidegrees/sec/sec
static constexpr double ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS = 110000.;
// default maximum acceleration for yaw axis in centidegrees/sec/sec
static constexpr double ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS = 27000.;
// body-frame rate controller timeout in seconds
static constexpr double ATTITUDE_RATE_CONTROLLER_TIMEOUT = 1.;
// body-frame rate controller maximum output (for roll-pitch axis)
static constexpr double ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX = 1.;
// body-frame rate controller maximum output (for yaw axis)
static constexpr double ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX = 1.;
// This is used to decay the rate I term to 5% in half a second.
static constexpr double ATTITUDE_RATE_RELAX_TC = 0.16;
// Thrust angle error above which yaw corrections are limited
static constexpr double ATTITUDE_THRUST_ERROR_ANGLE = 30. * kRadians;
// delta time in seconds for 400hz update rate
static constexpr double ATTITUDE_400HZ_DT = 0.0025;
// body-frame rate feedforward enabled by default
static constexpr uint32_t ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT = 1;
// Time constant used to limit lean angle so that vehicle does not lose altitude
static constexpr double ATTITUDE_CONTROL_ANGLE_LIMIT_TC_DEFAULT = 1.;
// Max throttle used to limit lean angle so that vehicle does not lose altitude
static constexpr double ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX = 0.8;

static constexpr double ATTITUDE_CONTROL_MIN_DEFAULT = 0.1;  // minimum throttle mix default
static constexpr double ATTITUDE_CONTROL_MAN_DEFAULT = 0.1;  // manual throttle mix default
static constexpr double ATTITUDE_CONTROL_MAX_DEFAULT = 0.5;  // maximum throttle mix default
static constexpr double ATTITUDE_CONTROL_MIN_LIMIT = 0.5;    // min throttle mix upper limit
static constexpr double ATTITUDE_CONTROL_MAN_LIMIT = 4.;     // man throttle mix upper limit
static constexpr double ATTITUDE_CONTROL_MAX = 5.;           // maximum throttle mix default

// ratio controlling the max throttle output during competing requests of low throttle from
// the pilot (or autopilot) and higher throttle for attitude control.  Higher favours
// Attitude over pilot input
static constexpr double ATTITUDE_CONTROL_THR_MIX_DEFAULT = 0.5;
// default angle-p/pd throttle boost threshold
static constexpr double ATTITUDE_CONTROL_THR_G_BOOST_THRESH = 1.;

// Default parameters
static constexpr double DEFAULT_ATC_MULTI_RATE_RP_P = 0.135;
static constexpr double DEFAULT_ATC_MULTI_RATE_RP_I = 0.135;
static constexpr double DEFAULT_ATC_MULTI_RATE_RP_D = 0.0036;
static constexpr double DEFAULT_ATC_MULTI_RATE_RP_IMAX = 0.5;
static constexpr double DEFAULT_ATC_MULTI_RATE_RP_FILT_HZ = 20.;
static constexpr double DEFAULT_ATC_MULTI_RATE_YAW_P = 0.18;
static constexpr double DEFAULT_ATC_MULTI_RATE_YAW_I = 0.018;
static constexpr double DEFAULT_ATC_MULTI_RATE_YAW_D = 0.;
static constexpr double DEFAULT_ATC_MULTI_RATE_YAW_IMAX = 0.5;
static constexpr double DEFAULT_ATC_MULTI_RATE_YAW_FILT_HZ = 2.5;
}  // namespace tobas_mr_arducopter
