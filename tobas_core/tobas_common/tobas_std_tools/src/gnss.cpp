// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_std_tools/gnss.hpp"

#include <chrono>
#include <ctime>

namespace ch = std::chrono;

namespace tobas
{
namespace st
{
long computeGpsDelayFromToW(uint32_t gps_tow_ms)
{
  // GPS time is ahead of UTC by the leap seconds inserted since 1980 (unchanged since 2017).
  constexpr auto kGpsUtcLeapSeconds = ch::seconds(18);

  // Get the current GPS time.
  const auto now = ch::system_clock::now() + kGpsUtcLeapSeconds;
  const auto now_c = ch::system_clock::to_time_t(now);
  const auto gps_time = std::gmtime(&now_c);

  // Calculate 00:00:00 on Sunday of the current week.
  gps_time->tm_sec = 0;
  gps_time->tm_min = 0;
  gps_time->tm_hour = 0;
  gps_time->tm_mday -= gps_time->tm_wday;  // Move back from the current weekday to Sunday.
  // Use `timegm()` because `mktime()` interprets the time as local time.
  const auto start_of_week = ch::system_clock::from_time_t(timegm(gps_time));

  // Calculate the elapsed time from the start of the week to now.
  const auto duration = now - start_of_week;
  const auto cur_tow_ms = ch::duration_cast<ch::milliseconds>(duration).count();

  // Calculate the difference between GPS ToW and the current ToW.
  const auto diff_ms = cur_tow_ms - gps_tow_ms;

  return diff_ms;
}
}  // namespace st
}  // namespace tobas
