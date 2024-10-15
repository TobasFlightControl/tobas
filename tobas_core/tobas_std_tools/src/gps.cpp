#include <chrono>
#include <ctime>

#include "../include/tobas_std_tools/gps.hpp"

using namespace std;

namespace tobas_std
{
long computeGPSDelayFromToW(uint32_t gps_tow_ms)
{
  // 現在のUTC時刻を取得
  const auto now = chrono::system_clock::now();
  const auto now_c = chrono::system_clock::to_time_t(now);
  const auto utc_time = gmtime(&now_c);

  // その週の日曜日0時0分0秒を計算
  utc_time->tm_sec = 0;
  utc_time->tm_min = 0;
  utc_time->tm_hour = 0;
  utc_time->tm_mday -= utc_time->tm_wday;  // 現在の曜日から日曜日に戻る
  const auto start_of_week = chrono::system_clock::from_time_t(mktime(utc_time));

  // 週のはじめから現在までの経過時間を計算
  const auto duration = now - start_of_week;
  const auto cur_tow_ms = chrono::duration_cast<chrono::milliseconds>(duration).count();

  // GPS TOWと現在のTOWの差を計算
  const auto diff_ms = cur_tow_ms - gps_tow_ms;

  return diff_ms;
}
}  // namespace tobas_std
