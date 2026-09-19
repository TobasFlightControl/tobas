// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>

#include <tobas_msgs/msg/gnss.hpp>

namespace ntrip
{
namespace nmea
{

// 緯度経度・高度・ステータス等からNMEA GGAセンテンス（$GPGGA,...*CS\r\n）を生成
std::string createGgaSentence(
  double latitude_deg,
  double longitude_deg,
  double altitude_msl = 0.0,
  double geoid_height = 0.0,
  uint8_t fix_quality = 1,
  uint8_t num_satellites = 8,
  double hdop = 1.0,
  const std::chrono::system_clock::time_point& time = std::chrono::system_clock::now());

// tobas_msgs::msg::Gnss メッセージからNMEA GGAセンテンスを生成
std::string createGgaSentence(const tobas_msgs::msg::Gnss& gnss);

// NMEAチェックサム（'$' と '*' 間のXOR）を計算
uint8_t calculateChecksum(const std::string_view& sentence);

}  // namespace nmea
}  // namespace ntrip
