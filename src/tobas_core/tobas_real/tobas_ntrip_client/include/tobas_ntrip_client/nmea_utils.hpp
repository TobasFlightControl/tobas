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
/**
 * Build an NMEA GGA sentence (`$GPGGA,...*CS\r\n`) from a position and its status.
 *
 * @param _latitude_deg Geodetic latitude [deg]
 * @param _longitude_deg Geodetic longitude [deg]
 * @param _altitude_msl Height above mean sea level [m]
 * @param _geoid_height Height of the geoid above the WGS 84 ellipsoid [m]
 * @param _fix_quality GGA quality indicator, e.g. 1 = GNSS fix, 4 = RTK fix, 5 = RTK float
 * @param _num_satellites Number of satellites used in the solution
 * @param _hdop Horizontal dilution of precision
 * @param _time UTC time stamped on the sentence
 * @return The sentence including the checksum and the trailing `\r\n`.
 */
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

/** Compute the NMEA checksum, which is the XOR of the bytes between `$` and `*`. */
uint8_t calculateChecksum(std::string_view _sentence);
}  // namespace tobas::ntrip::nmea
