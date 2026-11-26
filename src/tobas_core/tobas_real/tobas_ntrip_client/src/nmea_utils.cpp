// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ntrip_client/nmea_utils.hpp"

#include <cmath>
#include <cstdio>
#include <ctime>

namespace tobas::ntrip::nmea
{
std::string createGgaSentence(
  double _latitude_deg,
  double _longitude_deg,
  double _altitude_msl,
  double _geoid_height,
  uint8_t _fix_quality,
  uint8_t _num_satellites,
  double _hdop,
  const std::chrono::system_clock::time_point& _time)
{
  // Break the time stamp down into UTC fields.
  const auto duration = _time.time_since_epoch();
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration - seconds).count();

  const std::time_t time_t_value = seconds.count();
  std::tm utc_tm = {};
  gmtime_r(&time_t_value, &utc_tm);

  char time_buf[16];
  std::snprintf(
    time_buf,
    sizeof(time_buf),
    "%02d%02d%02d.%02d",
    utc_tm.tm_hour,
    utc_tm.tm_min,
    utc_tm.tm_sec,
    static_cast<int>(millis / 10));

  // Format the latitude as DDMM.MMMMM.
  char lat_buf[20];
  const char lat_dir = _latitude_deg >= 0.0 ? 'N' : 'S';
  const double lat_abs = std::abs(_latitude_deg);
  const int lat_deg = static_cast<int>(lat_abs);
  const double lat_min = (lat_abs - lat_deg) * 60.0;
  std::snprintf(lat_buf, sizeof(lat_buf), "%02d%08.5f", lat_deg, lat_min);

  // Format the longitude as DDDMM.MMMMM.
  char lon_buf[20];
  const char lon_dir = _longitude_deg >= 0.0 ? 'E' : 'W';
  const double lon_abs = std::abs(_longitude_deg);
  const int lon_deg = static_cast<int>(lon_abs);
  const double lon_min = (lon_abs - lon_deg) * 60.0;
  std::snprintf(lon_buf, sizeof(lon_buf), "%03d%08.5f", lon_deg, lon_min);

  // Build the payload, which is everything between `$` and `*`.
  char payload[160];
  std::snprintf(
    payload,
    sizeof(payload),
    "$GPGGA,%s,%s,%c,%s,%c,%u,%02u,%.1f,%.2f,M,%.2f,M,,",
    time_buf,
    lat_buf,
    lat_dir,
    lon_buf,
    lon_dir,
    static_cast<unsigned int>(_fix_quality),
    static_cast<unsigned int>(_num_satellites),
    _hdop,
    _altitude_msl,
    _geoid_height);

  // Append the checksum and the sentence terminator.
  const uint8_t checksum = calculateChecksum(payload);

  char full_sentence[180];
  std::snprintf(full_sentence, sizeof(full_sentence), "%s*%02X\r\n", payload, checksum);

  return std::string(full_sentence);
}

std::string createGgaSentence(const tobas_msgs::msg::Gnss& _gnss)
{
  uint8_t fix_quality = 0;
  if (_gnss.fix_type == tobas_msgs::msg::Gnss::RTK_FIX) {
    fix_quality = 4;
  }
  else if (_gnss.fix_type == tobas_msgs::msg::Gnss::RTK_FLOAT) {
    fix_quality = 5;
  }
  else if (_gnss.fix_type >= tobas_msgs::msg::Gnss::FIX_2D) {
    fix_quality = 1;
  }
  else {
    fix_quality = 0;
  }

  uint8_t num_satellites = _gnss.num_satellites_used;
  if (num_satellites == 0 && fix_quality > 0) {
    num_satellites = 8;
  }

  const double geoid_height = _gnss.height_wgs84 - _gnss.height_msl;

  // GGA requires UTC, but `header.stamp` is not UTC when the node clock is steady time (`TOBAS_CLOCK_TYPE=steady_time`)
  // or simulation time. Use the system clock instead; the position is at most one GNSS update period old.
  const auto utc_time = std::chrono::system_clock::now();

  return createGgaSentence(
    _gnss.latitude, _gnss.longitude, _gnss.height_msl, geoid_height, fix_quality, num_satellites, 1.0, utc_time);
}

uint8_t calculateChecksum(std::string_view _sentence)
{
  uint8_t checksum = 0;
  size_t start = 0;
  if (!_sentence.empty() && _sentence[0] == '$') {
    start = 1;
  }
  for (size_t i = start; i < _sentence.size(); ++i) {
    if (_sentence[i] == '*') {
      break;
    }
    checksum ^= static_cast<uint8_t>(_sentence[i]);
  }
  return checksum;
}
}  // namespace tobas::ntrip::nmea
