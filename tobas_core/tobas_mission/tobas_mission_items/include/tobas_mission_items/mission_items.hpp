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
};

struct PACKED Waypoint
{
  std::float64_t latitude;                 // [deg]
  std::float64_t longitude;                // [deg]
  std::float64_t altitude;                 // [m]
  std::float64_t max_horizontal_velocity;  // [m/s]
  std::float64_t max_vertical_velocity;    // [m/s]
  std::float64_t max_horizontal_accel;     // [m/s^2]
  std::float64_t max_vertical_accel;       // [m/s^2]
  std::float64_t max_horizontal_jerk;      // [m/s^3]
  std::float64_t max_vertical_jerk;        // [m/s^3]
  std::float64_t acceptance_radius;        // [m]
  std::float64_t altitude_tolerance;       // [m]
  std::float64_t timeout;                  // [s]
};

struct PACKED Takeoff
{
  std::float64_t altitude;            // [m]
  std::float64_t max_speed;           // [m/s]
  std::float64_t max_accel;           // [m/s^2]
  std::float64_t max_jerk;            // [m/s^3]
  std::float64_t altitude_tolerance;  // [m]
  std::float64_t timeout;             // [s]
};

struct PACKED Land
{
  std::float64_t speed;    // [m/s]
  std::float64_t timeout;  // [s]
};
}  // namespace mission
}  // namespace tobas
