#pragma once

#include <cinttypes>

namespace tobas_std
{
/* 与えられたGPS ToW (GPS Time of Week) とNTPサーバから得られたUTCから，GPSの遅延を計算する． */
long computeGPSDelayFromToW(uint32_t gps_tow_ms);

/**
 * @brief 緯度，経度，高度を三次元直行座標に変換する．
 * cf. https://qiita.com/Toramin10/items/fa0c8e79aaadf84ddb25
 *
 * @param latitude 緯度[deg]
 * @param longitude 経度[deg]
 * @param altitude 高度[m]
 * @param x,y,z 直行座標[m](出力)
 */
void gnssToCartAbsolute(
  const double& latitude,
  const double& longitude,
  const double& altitude,
  double& x,
  double& y,
  double& z);

/**
 * @brief 緯度，経度を平面直行座標に変換する．
 * cf. https://qiita.com/sw1227/items/e7a590994ad7dcd0e8ab
 *
 * @param latitude 北緯[deg]
 * @param longitude 東経[deg]
 * @param latitude_0 原点の北緯[deg]
 * @param longitude_0 原点の東経[deg]
 * @param east 東向きの座標[m] (出力)
 * @param north 北向きの座標[m] (出力)
 */
void gnssToCartRelative(
  const double& latitude,
  const double& longitude,
  const double& latitude_0,
  const double& longitude_0,
  double& east,
  double& north);

/**
 * @brief 平面直行座標を緯度，経度に変換する．
 * cf. https://qiita.com/sw1227/items/e7a590994ad7dcd0e8ab
 *
 * @param east 東向きの座標[m]
 * @param north 北向きの座標[m]
 * @param latitude_0 原点の北緯[deg]
 * @param longitude_0 原点の東経[deg]
 * @param latitude 北緯[deg] (出力)
 * @param longitude 東経[deg] (出力)
 */
void cartToGnssRelative(
  const double& east,
  const double& north,
  const double& latitude_0,
  const double& longitude_0,
  double& latitude,
  double& longitude);
}  // namespace tobas_std
