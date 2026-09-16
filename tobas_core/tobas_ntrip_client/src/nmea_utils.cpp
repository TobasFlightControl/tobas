#include "tobas_ntrip_client/nmea_utils.hpp"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace ntrip
{
namespace nmea
{

uint8_t calculateChecksum(const std::string_view& sentence)
{
  uint8_t checksum = 0;
  size_t start = 0;
  if (!sentence.empty() && sentence[0] == '$') {
    start = 1;
  }
  for (size_t i = start; i < sentence.size(); ++i) {
    if (sentence[i] == '*') {
      break;
    }
    checksum ^= static_cast<uint8_t>(sentence[i]);
  }
  return checksum;
}

std::string createGgaSentence(
  double latitude_deg,
  double longitude_deg,
  double altitude_msl,
  double geoid_height,
  uint8_t fix_quality,
  uint8_t num_satellites,
  double hdop,
  const std::chrono::system_clock::time_point& time)
{
  // UTC時刻の取得
  auto duration = time.time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
  auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration - seconds).count();

  std::time_t tt = seconds.count();
  std::tm utc_tm;
  gmtime_r(&tt, &utc_tm);

  char time_buf[16];
  std::snprintf(
    time_buf,
    sizeof(time_buf),
    "%02d%02d%02d.%02d",
    utc_tm.tm_hour,
    utc_tm.tm_min,
    utc_tm.tm_sec,
    static_cast<int>(millis / 10));

  // 緯度のフォーマット (DDMM.MMMMM)
  char lat_buf[20];
  char lat_dir = latitude_deg >= 0.0 ? 'N' : 'S';
  double lat_abs = std::abs(latitude_deg);
  int lat_deg = static_cast<int>(lat_abs);
  double lat_min = (lat_abs - lat_deg) * 60.0;
  std::snprintf(lat_buf, sizeof(lat_buf), "%02d%08.5f", lat_deg, lat_min);

  // 経度のフォーマット (DDDMM.MMMMM)
  char lon_buf[20];
  char lon_dir = longitude_deg >= 0.0 ? 'E' : 'W';
  double lon_abs = std::abs(longitude_deg);
  int lon_deg = static_cast<int>(lon_abs);
  double lon_min = (lon_abs - lon_deg) * 60.0;
  std::snprintf(lon_buf, sizeof(lon_buf), "%03d%08.5f", lon_deg, lon_min);

  // ペイロード作成 ($GPGGA,...)
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
    fix_quality,
    num_satellites,
    hdop,
    altitude_msl,
    geoid_height);

  // チェックサム計算
  uint8_t cs = calculateChecksum(payload);

  char full_sentence[180];
  std::snprintf(full_sentence, sizeof(full_sentence), "%s*%02X\r\n", payload, cs);

  return std::string(full_sentence);
}

std::string createGgaSentence(const tobas_msgs::msg::Gnss& gnss)
{
  uint8_t fix_quality = 0;
  if (gnss.rtk_status == tobas_msgs::msg::Gnss::RTK_FIXED) {
    fix_quality = 4;
  }
  else if (gnss.rtk_status == tobas_msgs::msg::Gnss::RTK_FLOAT) {
    fix_quality = 5;
  }
  else if (gnss.rtk_status == tobas_msgs::msg::Gnss::CORRECTION_RECEIVED) {
    fix_quality = 2;
  }
  else if (gnss.fix_type >= tobas_msgs::msg::Gnss::FIX_2D) {
    fix_quality = 1;
  }
  else {
    fix_quality = 0;
  }

  uint8_t num_sats = gnss.num_satellites_used;
  if (num_sats == 0 && fix_quality > 0) {
    num_sats = 8;
  }

  double geoid_height = gnss.height_wgs84 - gnss.height_msl;

  std::chrono::system_clock::time_point stamp_time;
  if (gnss.header.stamp.sec > 0) {
    stamp_time = std::chrono::system_clock::time_point(
      std::chrono::seconds(gnss.header.stamp.sec) + std::chrono::nanoseconds(gnss.header.stamp.nanosec));
  }
  else {
    stamp_time = std::chrono::system_clock::now();
  }

  return createGgaSentence(
    gnss.latitude, gnss.longitude, gnss.height_msl, geoid_height, fix_quality, num_sats, 1.0, stamp_time);
}

}  // namespace nmea
}  // namespace ntrip
