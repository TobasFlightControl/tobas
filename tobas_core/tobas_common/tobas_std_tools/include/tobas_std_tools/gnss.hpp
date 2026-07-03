// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cinttypes>
#include <tuple>

namespace tobas
{
namespace st
{
/* Calculate GPS delay from the given GPS ToW (GPS Time of Week) and UTC obtained from an NTP server. */
long computeGpsDelayFromToW(uint32_t gps_tow_ms);

/**
 * @brief Convert latitude, longitude, and altitude to 3D Cartesian coordinates.
 * cf. https://qiita.com/Toramin10/items/fa0c8e79aaadf84ddb25
 *
 * @param latitude Latitude [deg].
 * @param longitude Longitude [deg].
 * @param altitude Altitude [m].
 *
 * @return XYZ Cartesian coordinates [m].
 */
std::tuple<double, double, double>
gnssToCartAbsolute(const double& latitude, const double& longitude, const double& altitude);

/**
 * @brief Convert latitude and longitude to planar Cartesian coordinates.
 * cf. https://qiita.com/sw1227/items/e7a590994ad7dcd0e8ab
 *
 * @param latitude North latitude [deg].
 * @param longitude East longitude [deg].
 * @param latitude_0 Origin north latitude [deg].
 * @param longitude_0 Origin east longitude [deg].
 *
 * @return East coordinate [m], north coordinate [m].
 */
std::tuple<double, double>
gnssToCartRelative(const double& latitude, const double& longitude, const double& latitude_0, const double& longitude_0);

/**
 * @brief Convert planar Cartesian coordinates to latitude and longitude.
 * cf. https://qiita.com/sw1227/items/e7a590994ad7dcd0e8ab
 *
 * @param east East coordinate [m].
 * @param north North coordinate [m].
 * @param latitude_0 Origin north latitude [deg].
 * @param longitude_0 Origin east longitude [deg].
 *
 * @return North latitude [deg], east longitude [deg].
 */
std::tuple<double, double>
cartToGnssRelative(const double& east, const double& north, const double& latitude_0, const double& longitude_0);
}  // namespace st
}  // namespace tobas
