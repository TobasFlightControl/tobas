// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>

#include <tobas_msgs/msg/gnss.hpp>

namespace tobas::ntrip::nmea
{
/** Build an NMEA GGA sentence (`$GPGGA,...*CS\r\n`) from a position and its status. */
std::string createGgaSentence(
  double _latitude_deg,
  double _longitude_deg,
  double _altitude_msl = 0.0,
  double _geoid_height = 0.0,
  uint8_t _fix_quality = 1,
  uint8_t _num_satellites = 8,
  double _hdop = 1.0,
  const std::chrono::system_clock::time_point& _time = std::chrono::system_clock::now());

/** Build an NMEA GGA sentence from a GNSS message. */
std::string createGgaSentence(const tobas_msgs::msg::Gnss& _gnss);

/** Compute the NMEA checksum, the XOR of the bytes between `$` and `*`. */
uint8_t calculateChecksum(std::string_view _sentence);
}  // namespace tobas::ntrip::nmea
