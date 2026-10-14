// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <ctime>
#include <expected>
#include <string>
#include <thread>

namespace tobas::tim
{
inline void sleep(double _sec)
{
  std::this_thread::sleep_for(std::chrono::duration<double>(_sec));
}

inline void msleep(size_t _msec)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(_msec));
}

inline void usleep(size_t _usec)
{
  std::this_thread::sleep_for(std::chrono::microseconds(_usec));
}

inline void nsleep(size_t _nsec)
{
  std::this_thread::sleep_for(std::chrono::nanoseconds(_nsec));
}

template <int Hz>
inline constexpr std::chrono::duration<int, std::ratio<1, Hz>> periodFromFrequency()
{
  return std::chrono::duration<int, std::ratio<1, Hz>>{ 1 };
}

std::expected<std::chrono::system_clock::time_point, std::string> tmToTimePoint(tm _tm);
tm timePointToTm(const std::chrono::system_clock::time_point& _tp);

tm tmFromUTC(int _year, int _month, int _day, int _hour, int _min, int _sec);

std::expected<std::chrono::system_clock::time_point, std::string>
timePointFromUTC(int _year, int _month, int _day, int _hour, int _min, int _sec, int _nano);

/** Convert a Gregorian date to a fractional year using day precision. */
double yearFraction(const std::chrono::system_clock::time_point& _tp = std::chrono::system_clock::now());
}  // namespace tobas::tim
