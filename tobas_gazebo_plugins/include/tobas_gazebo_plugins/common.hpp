#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>

// Log throttle data structure
std::map<std::string, gazebo::common::Time> g_log_throttle;
// Mutex to protect logThrottle. Without it, the formatting breaks down.
std::mutex g_log_throttle_mutex;

#define GZ_LOG_THROTTLE(period, msg, os)                                                           \
  {                                                                                                \
    std::string id = std::string(__FILE__) + ":" + std::to_string(__LINE__);                       \
    const auto now = gazebo::common::Time::GetWallTime();                                          \
    std::lock_guard<std::mutex> lock(g_log_throttle_mutex);                                        \
    auto it = g_log_throttle.find(id);                                                             \
    if (it == g_log_throttle.end())                                                                \
    {                                                                                              \
      g_log_throttle[id] = now;                                                                    \
      os << msg << std::endl;                                                                      \
    }                                                                                              \
    else                                                                                           \
    {                                                                                              \
      const auto diff = (now - it->second).Double();                                               \
      if (diff > period)                                                                           \
      {                                                                                            \
        it->second = now;                                                                          \
        os << msg << std::endl;                                                                    \
      }                                                                                            \
    }                                                                                              \
  }

#define GZ_MSG_THROTTLE(period, msg) GZ_LOG_THROTTLE(period, msg, gzmsg)
#define GZ_WARN_THROTTLE(period, msg) GZ_LOG_THROTTLE(period, msg, gzwarn)
#define GZ_ERROR_THROTTLE(period, msg) GZ_LOG_THROTTLE(period, msg, gzerr)

namespace gazebo
{
using SdfVector2 = ignition::math::Vector2d;
using SdfVector3 = ignition::math::Vector3d;
using NormalDistribution = std::normal_distribution<double>;
using UniformDistribution = std::uniform_real_distribution<double>;

// Constants
static constexpr double kDegreeToRadian = M_PI / 180.;
static constexpr double kRadianToDegree = 1 / kDegreeToRadian;
static constexpr double kFeetToMeter = 0.3048;
static constexpr double kMeterToFeet = 1 / kFeetToMeter;

static const SdfVector3 zero3 = SdfVector3(0., 0., 0.);
static constexpr double kWarnPeriod = 3.;
static constexpr double kErrorPeriod = 1.;
static constexpr double kCheckTopicsTimeThreshold = 1.;  // [s]

// Default values
static const std::string kDefaultBatteryTopic = "battery";
static const std::string kDefaultWindTopic = "wind";

static constexpr double kDefaultLatitudeZero = 35.658099;    // 日本: 北緯35度39分29秒
static constexpr double kDefaultLongitudeZero = 139.741354;  // 日本: 東経139度44分28秒8759
static constexpr double kDefaultAltitudeZero = 0.;           // [m]

static constexpr double kDefaultRotorSpeedSlowdownSim = 10.;
static constexpr double kDefaultCheckDelayThreshold = 0.02;   // [s]
static constexpr double kDefaultAutoStopTimeThreshold = 0.5;  // [s]
}  // namespace gazebo
