// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_time_tools/util.hpp"

namespace ch = std::chrono;

namespace tobas
{
namespace tim
{
ch::system_clock::time_point tmToTimePoint(tm tm)
{
  // Convert `tm` represented in UTC to `time_t`.
  // https://dev.activebasic.com/egtra/2017/01/03/941/
  const auto tt = timegm(&tm);
  if (tt < 0) {
    throw std::runtime_error("Failed to convert tm to time_t.");
  }

  // The time zone does not affect conversion from `time_t` to `time_point`.
  return ch::system_clock::from_time_t(tt);
}

tm timePointToTm(const ch::system_clock::time_point& tp)
{
  const auto tt = ch::system_clock::to_time_t(tp);
  return *std::gmtime(&tt);
}

tm tmFromUTC(int year, int month, int day, int hour, int min, int sec)
{
  tm tm;
  tm.tm_year = year - 1900;  // Year is an offset from 1900.
  tm.tm_mon = month - 1;     // Month starts from 0.
  tm.tm_mday = day;
  tm.tm_hour = hour;
  tm.tm_min = min;
  tm.tm_sec = sec;
  return tm;
}

ch::system_clock::time_point timePointFromUTC(int year, int month, int day, int hour, int min, int sec, int nano)
{
  const auto tm = tmFromUTC(year, month, day, hour, min, sec);
  return tmToTimePoint(tm) + ch::nanoseconds(nano);
}

double yearFraction(const ch::system_clock::time_point& tp)
{
  const auto tm = timePointToTm(tp);
  const auto year = tm.tm_year + 1900;  // Year.
  const auto is_leap_year = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
  const auto day_of_year = tm.tm_yday;                 // Current day in the year, starting from 0.
  const auto days_in_year = is_leap_year ? 366 : 365;  // 366 days in a leap year.
  return year + static_cast<double>(day_of_year) / days_in_year;
}
}  // namespace tim
}  // namespace tobas
