#include "../include/tobas_std_tools/time.hpp"

using namespace std;
using namespace chrono;

namespace tobas_std
{
system_clock::time_point tmToTimePoint(tm tm)
{
  // UTCで表現されたtmをtime_tに変換
  // https://dev.activebasic.com/egtra/2017/01/03/941/
  const auto tt = timegm(&tm);
  if (tt == -1)
    throw runtime_error("Failed to convert tm to time_t.");

  // time_t -> time_pointの変換にはタイムゾーンは影響しない
  return system_clock::from_time_t(tt);
}

tm timePointToTm(const system_clock::time_point& tp)
{
  const auto tt = system_clock::to_time_t(tp);
  return *gmtime(&tt);
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

system_clock::time_point timePointFromUTC(int year, int month, int day, int hour, int min, int sec, int nano)
{
  const auto tm = tmFromUTC(year, month, day, hour, min, sec);
  return tmToTimePoint(tm) + nanoseconds(nano);
}

double yearFraction(const system_clock::time_point& tp)
{
  const auto tm = timePointToTm(tp);
  const auto year = tm.tm_year + 1900;  // 年
  const auto is_leap_year = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
  const auto day_of_year = tm.tm_yday;                 // 年の中の現在の日数（0から始まる）
  const auto days_in_year = is_leap_year ? 366 : 365;  // 閏年の場合は366日
  return year + static_cast<double>(day_of_year) / days_in_year;
}
}  // namespace tobas_std

ostream& operator<<(ostream& os, const tm& arg)
{
  os << "Year: " << arg.tm_year + 1900 << endl;  // Years since 1900
  os << "Month: " << arg.tm_mon + 1 << endl;     // Months since January [0-11]
  os << "Day: " << arg.tm_mday << endl;
  os << "Hour: " << arg.tm_hour << endl;
  os << "Min: " << arg.tm_min << endl;
  os << "Sec: " << arg.tm_sec << endl;
  return os;
}

double operator-(tm lhs, tm rhs)
{
  const auto time_l = mktime(&lhs);
  const auto time_r = mktime(&rhs);
  return difftime(time_l, time_r);  // sec
}
