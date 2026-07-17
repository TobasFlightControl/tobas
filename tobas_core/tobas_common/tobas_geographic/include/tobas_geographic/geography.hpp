// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/TransverseMercator.hpp>

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

class Geography final
{
public:
  explicit Geography(const std::string& name = "wmm2025");

  Geography(const Geography&) = delete;
  Geography& operator=(const Geography&) = delete;
  Geography(Geography&&) = delete;
  Geography& operator=(Geography&&) = delete;

  /**
   * @brief Project geodetic coordinates onto a GRS80 Gauss-Kruger plane.
   *
   * The origin is assigned zero easting and northing. The central scale factor is 0.9999.
   */
  PlaneCoordinates geodeticToPlane(double latitude, double longitude, double origin_latitude, double origin_longitude);

  /** @brief Reverse a GRS80 Gauss-Kruger projection with a zero-valued origin. */
  GeodeticCoordinates planeToGeodetic(double east, double north, double origin_latitude, double origin_longitude);

  /**
   * @brief Evaluate WMM2025 at the specified geodetic position and decimal year.
   *
   * @param latitude Geodetic latitude [deg].
   * @param longitude Geodetic longitude [deg].
   * @param height_wgs84 Height above the WGS 84 ellipsoid [m].
   * @param decimal_year Decimal year.
   */
  MagneticField magneticField(double latitude, double longitude, double height_wgs84, double decimal_year);

private:
  GeographicLib::TransverseMercator plane_projection_;
  GeographicLib::MagneticModel magnetic_model_;

  double origin_east_;   // Projected origin easting [m]
  double origin_north_;  // Projected origin northing [m]
  double east_;          // Easting [m] or magnetic east component [nT]
  double north_;         // Northing [m] or magnetic north component [nT]
  double up_;            // Magnetic up component [nT]
  double latitude_;      // Reverse-projected latitude [deg]
  double longitude_;     // Reverse-projected longitude [deg]
  double horizontal_;    // Horizontal magnetic intensity [nT]
  double total_;         // Total magnetic intensity [nT]
  double declination_;   // Magnetic declination [deg]
  double inclination_;   // Magnetic inclination [deg]

  bool model_expiration_warned_ = false;  // Whether the expiration warning has been emitted
};
}  // namespace geo
}  // namespace tobas
