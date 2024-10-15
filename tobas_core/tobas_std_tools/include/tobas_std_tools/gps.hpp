#pragma once

#include <cinttypes>

namespace tobas_std
{
/* 与えられたGPS ToW (GPS Time of Week) とNTPサーバから得られたUTCから，GPSの遅延を計算する． */
long computeGPSDelayFromToW(uint32_t gps_tow_ms);
}  // namespace tobas_std
