// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_geographic/geography.hpp"

#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/TransverseMercator.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace tobas
{
namespace geo
{
namespace
{
constexpr double kGrs80SemiMajorAxis = 6378137.0;         // [m]
constexpr double kGrs80Flattening = 1.0 / 298.257222101;  // [-]
constexpr double kCentralScale = 0.9999;                  // [-]
constexpr double kNanoTeslaToGauss = 1e-5;

const GeographicLib::TransverseMercator& planeProjection()
{
  static const GeographicLib::TransverseMercator projection(kGrs80SemiMajorAxis, kGrs80Flattening, kCentralScale);
  return projection;
}

const GeographicLib::MagneticModel& magneticModel()
{
  static const auto model_path = ament_index_cpp::get_package_share_directory("tobas_geographic") + "/data/magnetic";
  static const GeographicLib::MagneticModel model("wmm2025", model_path);
  return model;
}
}  // namespace

PlaneCoordinates geodeticToPlane(double latitude, double longitude, double origin_latitude, double origin_longitude)
{
  double origin_east;
  double origin_north;
  planeProjection().Forward(origin_longitude, origin_latitude, origin_longitude, origin_east, origin_north);

  double east;
  double north;
  planeProjection().Forward(origin_longitude, latitude, longitude, east, north);

  return { east - origin_east, north - origin_north };
}

GeodeticCoordinates planeToGeodetic(double east, double north, double origin_latitude, double origin_longitude)
{
  double origin_east;
  double origin_north;
  planeProjection().Forward(origin_longitude, origin_latitude, origin_longitude, origin_east, origin_north);

  double latitude;
  double longitude;
  planeProjection().Reverse(origin_longitude, east + origin_east, north + origin_north, latitude, longitude);
  
  return { latitude, longitude };
}

MagneticField magneticField(double latitude, double longitude, double ellipsoid_height, double decimal_year)
{
  double east;
  double north;
  double up;
  magneticModel()(decimal_year, latitude, longitude, ellipsoid_height, east, north, up);

  double horizontal;
  double total;
  double declination;
  double inclination;
  GeographicLib::MagneticModel::FieldComponents(east, north, up, horizontal, total, declination, inclination);

  return { east * kNanoTeslaToGauss, north * kNanoTeslaToGauss, up * kNanoTeslaToGauss, total * kNanoTeslaToGauss };
}
}  // namespace geo
}  // namespace tobas
