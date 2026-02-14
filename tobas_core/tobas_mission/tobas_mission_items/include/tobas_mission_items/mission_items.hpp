#pragma once

#include <cinttypes>
#include <stdfloat>

#define PACKED __attribute__((__packed__))  // 構造体のメンバ変数がメモリ上で連続する

namespace tobas
{
namespace mission
{
enum Type : uint8_t
{
  kWaypoint,
  kTakeoff,
  kLand,
  kReturnToLaunch,
};

enum AltitudeFrame : uint8_t
{
  kRelativeToHome,
  kMeanSeaLevel,
};

struct PACKED Waypoint
{
  std::float64_t latitude = 0.;   // [deg]
  std::float64_t longitude = 0.;  // [deg]

  std::float64_t altitude = 0.;  // [m]
  AltitudeFrame altitude_frame = kRelativeToHome;

  bool auto_heading = true;

  std::float64_t max_horizontal_velocity = 0.;  // [m/s]
  std::float64_t max_vertical_velocity = 0.;    // [m/s]
  std::float64_t max_horizontal_accel = 0.;     // [m/s^2]
  std::float64_t max_vertical_accel = 0.;       // [m/s^2]
  std::float64_t max_horizontal_jerk = 0.;      // [m/s^3]
  std::float64_t max_vertical_jerk = 0.;        // [m/s^3]

  std::float64_t acceptance_radius = 0.;   // [m]
  std::float64_t altitude_tolerance = 0.;  // [m]

  std::float64_t timeout = 0.;  // [s]
};

struct PACKED Takeoff
{
  std::float64_t altitude = 0.;  // [m]
  AltitudeFrame altitude_frame = kRelativeToHome;

  std::float64_t max_speed = 0.;  // [m/s]
  std::float64_t max_accel = 0.;  // [m/s^2]
  std::float64_t max_jerk = 0.;   // [m/s^3]

  std::float64_t altitude_tolerance = 0.;  // [m]

  std::float64_t timeout = 0.;  // [s]
};

struct PACKED Land
{
  std::float64_t speed = 0.;  // [m/s]

  std::float64_t timeout = 0.;  // [s]
};

struct PACKED ReturnToLaunch
{
  std::float64_t min_altitude = 0.;  // [m]

  bool auto_heading = true;

  std::float64_t max_horizontal_velocity = 0.;  // [m/s]
  std::float64_t max_vertical_velocity = 0.;    // [m/s]
  std::float64_t max_horizontal_accel = 0.;     // [m/s^2]
  std::float64_t max_vertical_accel = 0.;       // [m/s^2]
  std::float64_t max_horizontal_jerk = 0.;      // [m/s^3]
  std::float64_t max_vertical_jerk = 0.;        // [m/s^3]

  std::float64_t acceptance_radius = 0.;   // [m]
  std::float64_t altitude_tolerance = 0.;  // [m]

  std::float64_t timeout = 0.;  // [s]
};
}  // namespace mission
}  // namespace tobas
