// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cinttypes>
#include <stdfloat>

#include "./altitude_frame.hpp"

#define PACKED __attribute__((__packed__))  // Struct member variables are stored contiguously in memory.

namespace tobas
{
namespace mission
{
struct PACKED Waypoint
{
  std::float64_t latitude = 0.0;   // [deg]
  std::float64_t longitude = 0.0;  // [deg]

  std::float64_t altitude = 0.0;  // [m]
  AltitudeFrame altitude_frame = kRelativeToLaunch;

  bool auto_heading = true;
  bool stop_at_waypoint = false;

  std::float64_t max_horizontal_velocity = 0.0;  // [m/s]
  std::float64_t max_horizontal_accel = 0.0;     // [m/s^2]
  std::float64_t max_horizontal_jerk = 0.0;      // [m/s^3]
  std::float64_t max_vertical_velocity = 0.0;    // [m/s]
  std::float64_t max_vertical_accel = 0.0;       // [m/s^2]
  std::float64_t max_vertical_jerk = 0.0;        // [m/s^3]
  std::float64_t max_heading_rate = 0.0;         // [rad/s]
  std::float64_t max_heading_accel = 0.0;        // [rad/s^2]

  std::float64_t acceptance_radius = 0.0;   // [m]
  std::float64_t altitude_tolerance = 0.0;  // [m]

  std::float64_t timeout = 0.0;  // [s]
};

struct PACKED Takeoff
{
  std::float64_t altitude = 0.0;  // [m]
  AltitudeFrame altitude_frame = kRelativeToLaunch;

  std::float64_t max_speed = 0.0;  // [m/s]
  std::float64_t max_accel = 0.0;  // [m/s^2]
  std::float64_t max_jerk = 0.0;   // [m/s^3]

  std::float64_t altitude_tolerance = 0.0;  // [m]

  std::float64_t timeout = 0.0;  // [s]
};

struct PACKED Land
{
  std::float64_t speed = 0.0;  // [m/s]

  std::float64_t timeout = 0.0;  // [s]
};

struct PACKED ReturnToLaunch
{
  std::float64_t min_altitude = 0.0;  // [m]

  std::float64_t max_horizontal_velocity = 0.0;  // [m/s]
  std::float64_t max_horizontal_accel = 0.0;     // [m/s^2]
  std::float64_t max_horizontal_jerk = 0.0;      // [m/s^3]
  std::float64_t max_vertical_velocity = 0.0;    // [m/s]
  std::float64_t max_vertical_accel = 0.0;       // [m/s^2]
  std::float64_t max_vertical_jerk = 0.0;        // [m/s^3]
  std::float64_t max_heading_rate = 0.0;         // [rad/s]
  std::float64_t max_heading_accel = 0.0;        // [rad/s^2]

  std::float64_t acceptance_radius = 0.0;   // [m]
  std::float64_t altitude_tolerance = 0.0;  // [m]

  std::float64_t timeout = 0.0;  // [s]
};
}  // namespace mission
}  // namespace tobas

#undef PACKED
