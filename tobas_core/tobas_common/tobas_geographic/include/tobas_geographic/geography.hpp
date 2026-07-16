// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace geo
{
struct PlaneCoordinates
{
  double east;   // [m]
  double north;  // [m]
};

struct GeodeticCoordinates
{
  double latitude;   // [deg]
  double longitude;  // [deg]
};

struct MagneticField
{
  double east;   // [G]
  double north;  // [G]
  double up;     // [G]
  double total;  // [G]
};

/**
 * @brief Project geodetic coordinates onto a GRS80 Gauss-Kruger plane.
 *
 * The origin is assigned zero easting and northing. The central scale factor is 0.9999.
 */
PlaneCoordinates geodeticToPlane(double latitude, double longitude, double origin_latitude, double origin_longitude);

/**
 * @brief Reverse a GRS80 Gauss-Kruger projection with a zero-valued origin.
 */
GeodeticCoordinates planeToGeodetic(double east, double north, double origin_latitude, double origin_longitude);

/**
 * @brief Evaluate WMM2025 at the specified geodetic position and decimal year.
 *
 * @param latitude Geodetic latitude [deg].
 * @param longitude Geodetic longitude [deg].
 * @param ellipsoid_height Height above the WGS 84 ellipsoid [m].
 * @param decimal_year Decimal year.
 */
MagneticField magneticField(double latitude, double longitude, double ellipsoid_height, double decimal_year);
}  // namespace geo
}  // namespace tobas
