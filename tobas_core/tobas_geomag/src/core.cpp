#include "../include/tobas_geomag/core.hpp"

#include <cmath>

namespace geomag
{
Elements elementsFromMagField(const Vector& mag_field_itrs, double lat, double lon)
{
  const double x = mag_field_itrs.x * 1e+9;
  const double y = mag_field_itrs.y * 1e+9;
  const double z = mag_field_itrs.z * 1e+9;
  const double phi = lat * (M_PI / 180.);
  const double lam = lon * (M_PI / 180.);
  const double sphi = sin(phi);
  const double cphi = cos(phi);
  const double slam = sin(lam);
  const double clam = cos(lam);
  const double x1 = clam * x + slam * y;
  const double north = -sphi * x1 + cphi * z;
  const double east = -slam * x + clam * y;
  const double down = -cphi * x1 + -sphi * z;
  const double horizontal = sqrt(north * north + east * east);
  const double total = sqrt(horizontal * horizontal + down * down);
  const double inclination = atan2(down, horizontal) * (180. / M_PI);
  const double declination = atan2(east, north) * (180. / M_PI);
  return { north, east, down, horizontal, total, inclination, declination };
}

Vector ecefFromGeodetic(double lat, double lon, double h)
{
  // Convert to radians
  const double phi = lat * (M_PI / 180.);
  const double lam = lon * (M_PI / 180.);

  // WGS 84 constants
  constexpr double a = 6378137.;
  constexpr double f = 1. / 298.257223563;
  constexpr double e2 = f * (2 - f);
  constexpr double e2m = (1 - f) * (1 - f);

  const double sphi = sinf(phi);
  const double cphi = cosf(phi);
  const double slam = sinf(lam);
  const double clam = cosf(lam);
  const double n = a / sqrt(1. - e2 * (sphi * sphi));
  const double z = (e2m * n + h) * sphi;
  const double r = (n + h) * cphi;
  return { r * clam, r * slam, z };
}

Vector magFieldFromECEF(double dyear, const Vector& position_itrs, const ConstModel& WMM)
{
  // Mean radius of  ellipsoid in meters from section 1.2 of the WMM2015 Technical report
  constexpr double EARTH_R = 6371200.;

  const double x = position_itrs.x;
  const double y = position_itrs.y;
  const double z = position_itrs.z;
  const double rsqrd = x * x + y * y + z * z;

  double px = 0.;
  double py = 0.;
  double pz = 0.;

  double temp = EARTH_R / rsqrd;
  const double a = x * temp;
  const double b = y * temp;
  const double f = z * temp;
  const double g = EARTH_R * temp;

  // First m==0 row, just solve for the Vs
  double Vtop = EARTH_R / sqrt(rsqrd);  // V0,0
  double Wtop = 0.;                     // W0,0
  double Vprev = 0.;
  double Wprev = 0.;
  double Vnm = Vtop;
  double Wnm = Wtop;

  // Iterate through all ms
  for (size_t m = 0; m <= NMAX + 1; ++m)
  {
    // Iterate through all ns
    for (size_t n = m; n <= NMAX + 1; ++n)
    {
      if (n == m)
      {
        if (m != 0)
        {
          temp = Vtop;
          Vtop = (2 * m - 1) * (a * Vtop - b * Wtop);
          Wtop = (2 * m - 1) * (a * Wtop + b * temp);
          Vprev = 0;
          Wprev = 0;
          Vnm = Vtop;
          Wnm = Wtop;
        }
      }
      else
      {
        temp = Vnm;
        const double invs_temp = 1. / ((n - m));
        Vnm = ((2 * n - 1) * f * Vnm - (n + m - 1) * g * Vprev) * invs_temp;
        Vprev = temp;
        temp = Wnm;
        Wnm = ((2 * n - 1) * f * Wnm - (n + m - 1) * g * Wprev) * invs_temp;
        Wprev = temp;
      }
      if (m < NMAX && n >= m + 2)
      {
        px += 0.5 * (n - m) * (n - m - 1) * (WMM.C(n - 1, m + 1, dyear) * Vnm + WMM.S(n - 1, m + 1, dyear) * Wnm);
        py += 0.5 * (n - m) * (n - m - 1) * (-WMM.C(n - 1, m + 1, dyear) * Wnm + WMM.S(n - 1, m + 1, dyear) * Vnm);
      }
      if (n >= 2 && m >= 2)
      {
        px += 0.5 * (-WMM.C(n - 1, m - 1, dyear) * Vnm - WMM.S(n - 1, m - 1, dyear) * Wnm);
        py += 0.5 * (-WMM.C(n - 1, m - 1, dyear) * Wnm + WMM.S(n - 1, m - 1, dyear) * Vnm);
      }
      if (m == 1 && n >= 2)
      {
        px += -WMM.C(n - 1, 0, dyear) * Vnm;
        py += -WMM.C(n - 1, 0, dyear) * Wnm;
      }
      if (n >= 2 && n > m)
      {
        pz += (n - m) * (-WMM.C(n - 1, m, dyear) * Vnm - WMM.S(n - 1, m, dyear) * Wnm);
      }
    }
  }
  return { -px * 1e-9, -py * 1e-9, -pz * 1e-9 };
}
}  // namespace geomag
