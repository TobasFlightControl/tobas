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
  inline double C(size_t n, size_t m, double dyear) const
  {
    const auto index = (m * (2 * NMAX - m + 1)) / 2 + n;
    return Main_Field_Coeff_C[index] + (dyear - epoch) * Secular_Var_Coeff_C[index];
  }

  /* Function for indexing the S spherical component n,m at dyear time. */
  inline double S(size_t n, size_t m, double dyear) const
  {
    const auto index = (m * (2 * NMAX - m + 1)) / 2 + n;
    return Main_Field_Coeff_S[index] + (dyear - epoch) * Secular_Var_Coeff_S[index];
  }
};
}  // namespace geomag
