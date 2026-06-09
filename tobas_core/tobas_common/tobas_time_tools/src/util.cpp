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
  // UTCで表現されたtmをtime_tに変換
  // https://dev.activebasic.com/egtra/2017/01/03/941/
  const auto tt = timegm(&tm);
  if (tt < 0) {
    throw std::runtime_error("Failed to convert tm to time_t.");
  }

  // time_t -> time_pointの変換にはタイムゾーンは影響しない
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
  tm.tm_year = year - 1900;  // 年は1900からの差分
  tm.tm_mon = month - 1;     // 月は0から始まる
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
  const auto year = tm.tm_year + 1900;  // 年
  const auto is_leap_year = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
  const auto day_of_year = tm.tm_yday;                 // 年の中の現在の日数 (0から始まる)
  const auto days_in_year = is_leap_year ? 366 : 365;  // 閏年の場合は366日
  return year + static_cast<double>(day_of_year) / days_in_year;
}
}  // namespace tim
}  // namespace tobas
