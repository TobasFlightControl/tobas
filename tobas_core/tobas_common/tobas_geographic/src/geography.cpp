// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_geographic/geography.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tobas_std_tools/console.hpp>

namespace tobas
{
namespace geo
{
namespace
{
constexpr double kGrs80SemiMajorAxis = 6378137.0;         // [m]
constexpr double kGrs80Flattening = 1.0 / 298.257222101;  // [-]
constexpr double kCentralScale = 0.9999;                  // [-]
}  // namespace

Geography::Geography(const std::string& _name)
  : plane_projection_(kGrs80SemiMajorAxis, kGrs80Flattening, kCentralScale)
  , magnetic_model_(_name, ament_index_cpp::get_package_share_directory("tobas_geographic") + "/data/magnetic")
{
}

PlaneCoordinates
Geography::geodeticToPlane(double _latitude, double _longitude, double _origin_latitude, double _origin_longitude)
{
  plane_projection_.Forward(_origin_longitude, _origin_latitude, _origin_longitude, origin_east_, origin_north_);
  plane_projection_.Forward(_origin_longitude, _latitude, _longitude, east_, north_);
  return { east_ - origin_east_, north_ - origin_north_ };
}

GeodeticCoordinates
Geography::planeToGeodetic(double _east, double _north, double _origin_latitude, double _origin_longitude)
{
  plane_projection_.Forward(_origin_longitude, _origin_latitude, _origin_longitude, origin_east_, origin_north_);
  plane_projection_.Reverse(_origin_longitude, _east + origin_east_, _north + origin_north_, latitude_, longitude_);
  return { latitude_, longitude_ };
}

MagneticField Geography::magneticField(double _latitude, double _longitude, double _height_wgs84, double _decimal_year)
{
  if (_decimal_year > magnetic_model_.MaxTime() && !model_expiration_warned_) {
    PRINT_WARN(
      "Magnetic model '" << magnetic_model_.MagneticModelName() << "' expired at decimal year "
                         << magnetic_model_.MaxTime() << "; requested year is " << _decimal_year
                         << ". Update the bundled magnetic model data.");
    model_expiration_warned_ = true;
  }

  magnetic_model_(_decimal_year, _latitude, _longitude, _height_wgs84, east_, north_, up_);
  GeographicLib::MagneticModel::FieldComponents(east_, north_, up_, horizontal_, total_, declination_, inclination_);

  constexpr double kNanoTeslaToGauss = 1e-5;
  return { east_ * kNanoTeslaToGauss, north_ * kNanoTeslaToGauss, up_ * kNanoTeslaToGauss, total_ * kNanoTeslaToGauss };
}
}  // namespace geo
}  // namespace tobas
