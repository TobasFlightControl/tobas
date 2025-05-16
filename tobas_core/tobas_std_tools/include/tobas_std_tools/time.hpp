#pragma once

#include <chrono>
#include <iostream>
#include <thread>

namespace tobas_std
{
inline constexpr double secondsFromMilliSeconds(const int& msec)
{
  return msec / 1000;
}

inline constexpr int milliSecondsFromSeconds(const double& sec)
{
  return sec * 1000;
}

inline constexpr double secondsFromMicroSeconds(const int& usec)
{
  return usec / 1'000'000;
}

inline constexpr int microSecondsFromSeconds(const double& sec)
{
  return sec * 1'000'000;
}

inline void sleep(double sec)
{
  std::this_thread::sleep_for(std::chrono::duration<double>(sec));
}

inline void msleep(size_t msec)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(msec));
}

inline void usleep(size_t usec)
{
  std::this_thread::sleep_for(std::chrono::microseconds(usec));
}

inline void nsleep(size_t nsec)
{
  std::this_thread::sleep_for(std::chrono::nanoseconds(nsec));
}

std::chrono::system_clock::time_point tmToTimePoint(tm tm);
tm timePointToTm(const std::chrono::system_clock::time_point& tp);

tm tmFromUTC(int year, int month, int day, int hour, int min, int sec);

std::chrono::system_clock::time_point
timePointFromUTC(int year, int month, int day, int hour, int min, int sec, int nano);

/* 西暦を日数まで年に換算する． */
double yearFraction(const std::chrono::system_clock::time_point& tp = std::chrono::system_clock::now());
}  // namespace tobas_std

std::ostream& operator<<(std::ostream& os, const tm& arg);

double operator-(tm lhs, tm rhs);

template <typename Rep, typename Period>
std::ostream& operator<<(std::ostream& os, const std::chrono::duration<Rep, Period>& d)
{
  os << d.count() << " ";
  if constexpr (std::is_same_v<Period, std::ratio<1>>) {
    os << "s";
  }
  else if constexpr (std::is_same_v<Period, std::milli>) {
    os << "ms";
  }
  else if constexpr (std::is_same_v<Period, std::micro>) {
    os << "μs";
  }
  else if constexpr (std::is_same_v<Period, std::nano>) {
    os << "ns";
  }
  return os;
}
