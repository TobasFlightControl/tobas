// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_geographic/geography.hpp"

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
}  // namespace

Geography::Geography()
  : plane_projection_(kGrs80SemiMajorAxis, kGrs80Flattening, kCentralScale)
  , magnetic_model_("wmm2025", ament_index_cpp::get_package_share_directory("tobas_geographic") + "/data/magnetic")
{
}

PlaneCoordinates
Geography::geodeticToPlane(double latitude, double longitude, double origin_latitude, double origin_longitude)
{
  plane_projection_.Forward(origin_longitude, origin_latitude, origin_longitude, origin_east_, origin_north_);
  plane_projection_.Forward(origin_longitude, latitude, longitude, east_, north_);
  return { east_ - origin_east_, north_ - origin_north_ };
}

GeodeticCoordinates
Geography::planeToGeodetic(double east, double north, double origin_latitude, double origin_longitude)
{
  plane_projection_.Forward(origin_longitude, origin_latitude, origin_longitude, origin_east_, origin_north_);
  plane_projection_.Reverse(origin_longitude, east + origin_east_, north + origin_north_, latitude_, longitude_);
  return { latitude_, longitude_ };
}

MagneticField Geography::magneticField(double latitude, double longitude, double height_wgs84, double decimal_year)
{
  magnetic_model_(decimal_year, latitude, longitude, height_wgs84, east_, north_, up_);
  GeographicLib::MagneticModel::FieldComponents(east_, north_, up_, horizontal_, total_, declination_, inclination_);
  return { east_ * kNanoTeslaToGauss, north_ * kNanoTeslaToGauss, up_ * kNanoTeslaToGauss, total_ * kNanoTeslaToGauss };
}
}  // namespace geo
}  // namespace tobas
