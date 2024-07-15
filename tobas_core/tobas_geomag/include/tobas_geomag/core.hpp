#pragma once

#include <cstddef>

namespace geomag
{
constexpr size_t NMAX = 12;                             // Order of the Model
constexpr size_t NUMCOF = (NMAX + 1) * (NMAX + 2) / 2;  // Number of coefficents

struct Vector
{
  double x;
  double y;
  double z;
};

struct Elements
{
  double north;       // Local north magnetic field (nT)
  double east;        // Local east magnetic field (nT)
  double down;        // Local down magnetic field (nT)
  double horizontal;  // Local horizontal magnetic field intensity (nT)
  double total;       // Local total magnetic field intensity (nT)
  // Also called the dip angle,
  // the angle measured from the horizontal plane to the magnetic field vector;
  // a downward field is positive (deg)
  double inclination;
  // Also called the magnetic variation,
  // the angle between true north and the horizontal component of the field,
  // a eastward magnetic field of true North is positive (deg)
  double declination;
};

struct ConstModel
{
  double epoch;  // Decimal year
  double Main_Field_Coeff_C[NUMCOF];
  double Main_Field_Coeff_S[NUMCOF];
  double Secular_Var_Coeff_C[NUMCOF];
  double Secular_Var_Coeff_S[NUMCOF];

  /* Function for indexing the C spherical component n,m at dyear time. */
  double C(size_t n, size_t m, double dyear) const
  {
    const auto index = (m * (2 * NMAX - m + 1)) / 2 + n;
    return Main_Field_Coeff_C[index] + (dyear - epoch) * Secular_Var_Coeff_C[index];
  }

  /* Function for indexing the S spherical component n,m at dyear time. */
  double S(size_t n, size_t m, double dyear) const
  {
    const auto index = (m * (2 * NMAX - m + 1)) / 2 + n;
    return Main_Field_Coeff_S[index] + (dyear - epoch) * Secular_Var_Coeff_S[index];
  }
};

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
}  // namespace geomag
