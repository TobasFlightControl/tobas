#pragma once

#include "./model_params.hpp"
#include "./structs.hpp"

namespace geomag
{
/**
 * @brief Return a struct containing the 7 magnetic elements.
 * See https://www.geomag.nrcan.gc.ca/mag_fld/comp-en.php and https://www.ngdc.noaa.gov/geomag/icons/faqelems.gif
 * for more info.
 *
 * @param mag_field_itrs Local magnetic field in the itrs coordinate system (T).
 * @param lat Latitude in degrees, -90 at the south pole, 90 at the north pole.
 * @param lon Longitude in degrees.
 * @return Elements
 */
Elements elementsFromMagField(const Vector& mag_field_itrs, double lat, double lon);

/**
 * @brief Return the position in International Terrestrial Reference System coordinates, units meters.
 * Using the WGS 84 ellipsoid and the algorithm from https://geographiclib.sourceforge.io/
 *
 * @param lat Geodetic latitude in degrees, -90 at the south pole, 90 at the north pole.
 * @param lon Geodetic longitude in degrees.
 * @param h Height above the WGS 84 ellipsoid in meters.
 * @return Vector
 */
Vector ecefFromGeodetic(double lat, double lon, double h);

/**
 * @brief Return the magnetic field in International Terrestrial Reference System coordinates, units Tesla.
 *
 * @param dyear The decimal year, for example 2015.0.
 * @param position_itrs The location where the field is predicted, units m.
 * @param WMM Magnetic field model to use.
 * @return Vector
 */
Vector magFieldFromECEF(double dyear, const Vector& position_itrs, const ConstModel& WMM);

/**
 * @brief Return a struct containing the 7 magnetic elements.
 * See https://www.geomag.nrcan.gc.ca/mag_fld/comp-en.php and https://www.ngdc.noaa.gov/geomag/icons/faqelems.gif
 * for more info.
 *
 * @param lat Latitude in degrees, -90 at the south pole, 90 at the north pole.
 * @param lon Longitude in degrees.
 * @param h Height above the WGS 84 ellipsoid in meters.
 * @param dyear The decimal year, for example 2015.0.
 * @param WMM Magnetic field model to use.
 * @return Elements
 */
Elements elementsFromGeodetic(double lat, double lon, double h, double dyear, const ConstModel& WMM = WMM2025);
}  // namespace geomag
