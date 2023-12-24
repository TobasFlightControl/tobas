#include <chrono>
#include <iostream>

namespace dh_std
{
constexpr inline double secondsFromMilliSeconds(const int& ms)
{
  return ms / 1000;
}

constexpr inline int milliSecondsFromSeconds(const double& s)
{
  return s * 1000;
}

constexpr inline double secondsFromMicroSeconds(const int& us)
{
  return us / 1000000;
}

constexpr inline int microSecondsFromSeconds(const double& s)
{
  return s * 1000000;
}

std::chrono::system_clock::time_point tmToTimePoint(tm tm);
tm timePointToTm(const std::chrono::system_clock::time_point& tp);

tm tmFromUTC(int year, int month, int day, int hour, int min, int sec);

std::chrono::system_clock::time_point
timePointFromUTC(int year, int month, int day, int hour, int min, int sec, int nano);

/* 西暦を日数まで年に換算する． */
double
yearFraction(const std::chrono::system_clock::time_point& tp = std::chrono::system_clock::now());
}  // namespace dh_std

std::ostream& operator<<(std::ostream& os, const tm& arg);

double operator-(tm lhs, tm rhs);
