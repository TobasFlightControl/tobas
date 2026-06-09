// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_geomag/core.hpp"

#include <cmath>
#include <iostream>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace geomag
{
Elements elementsFromMagField(const Vector& mag_field_itrs, double lat, double lon)
{
  const auto& x = mag_field_itrs.x;                                     // [G]
  const auto& y = mag_field_itrs.y;                                     // [G]
  const auto& z = mag_field_itrs.z;                                     // [G]
  const auto phi = st::deg2rad(lat);                                    // [rad]
  const auto lam = st::deg2rad(lon);                                    // [rad]
  const auto sphi = std::sin(phi);                                      // [-]
  const auto cphi = std::cos(phi);                                      // [-]
  const auto slam = std::sin(lam);                                      // [-]
  const auto clam = std::cos(lam);                                      // [-]
  const auto x1 = clam * x + slam * y;                                  // [G]
  const auto north = -sphi * x1 + cphi * z;                             // [G]
  const auto east = -slam * x + clam * y;                               // [G]
  const auto down = -cphi * x1 + -sphi * z;                             // [G]
  const auto horizontal = std::sqrt(north * north + east * east);       // [G]
  const auto total = std::sqrt(horizontal * horizontal + down * down);  // [G]
  const auto inclination = st::rad2deg(std::atan2(down, horizontal));   // [deg]
  const auto declination = st::rad2deg(std::atan2(east, north));        // [deg]
  return { north, east, down, horizontal, total, inclination, declination };
}

Vector ecefFromGeodetic(double lat, double lon, double h)
{
  // Convert to radians
  const auto phi = st::deg2rad(lat);
  const auto lam = st::deg2rad(lon);

  // WGS 84 constants
  constexpr double a = 6378137.;  // [m]
  constexpr double f = 1. / 298.257223563;
  constexpr double e2 = f * (2 - f);
  constexpr double e2m = (1 - f) * (1 - f);

  const auto sphi = std::sin(phi);
  const auto cphi = std::cos(phi);
  const auto slam = std::sin(lam);
  const auto clam = std::cos(lam);
  const auto n = a / std::sqrt(1. - e2 * (sphi * sphi));
  const auto z = (e2m * n + h) * sphi;
  const auto r = (n + h) * cphi;
  return { r * clam, r * slam, z };
}

Vector magFieldFromECEF(double dyear, const Vector& position_itrs, const ConstModel& WMM)
{
  // Mean radius of  ellipsoid in meters from section 1.2 of the WMM2015 Technical report
  constexpr double EARTH_R = 6371200.;

  const auto& x = position_itrs.x;
  const auto& y = position_itrs.y;
  const auto& z = position_itrs.z;
  const auto rsqrd = math::sqr(x) + math::sqr(y) + math::sqr(z);

  double px = 0.;
  double py = 0.;
  double pz = 0.;

  auto temp = EARTH_R / rsqrd;
  const auto a = x * temp;
  const auto b = y * temp;
  const auto f = z * temp;
  const auto g = EARTH_R * temp;

  // First m==0 row, just solve for the Vs
  auto Vtop = EARTH_R / std::sqrt(rsqrd);  // V0,0
  double Wtop = 0.;                        // W0,0
  double Vprev = 0.;
  double Wprev = 0.;
  auto Vnm = Vtop;
  auto Wnm = Wtop;

  // Iterate through all ms
  for (size_t m = 0; m <= NMAX + 1; ++m) {
    // Iterate through all ns
    for (size_t n = m; n <= NMAX + 1; ++n) {
      if (n == m) {
        if (m != 0) {
          temp = Vtop;
          Vtop = (2 * m - 1) * (a * Vtop - b * Wtop);
          Wtop = (2 * m - 1) * (a * Wtop + b * temp);
          Vprev = 0;
          Wprev = 0;
          Vnm = Vtop;
          Wnm = Wtop;
        }
      }
      else {
        temp = Vnm;
        const auto invs_temp = 1. / ((n - m));
        Vnm = ((2 * n - 1) * f * Vnm - (n + m - 1) * g * Vprev) * invs_temp;
        Vprev = temp;
        temp = Wnm;
        Wnm = ((2 * n - 1) * f * Wnm - (n + m - 1) * g * Wprev) * invs_temp;
        Wprev = temp;
      }
      if (m < NMAX && n >= m + 2) {
        px += 0.5 * (n - m) * (n - m - 1) * (WMM.C(n - 1, m + 1, dyear) * Vnm + WMM.S(n - 1, m + 1, dyear) * Wnm);
        py += 0.5 * (n - m) * (n - m - 1) * (-WMM.C(n - 1, m + 1, dyear) * Wnm + WMM.S(n - 1, m + 1, dyear) * Vnm);
      }
      if (n >= 2 && m >= 2) {
        px += 0.5 * (-WMM.C(n - 1, m - 1, dyear) * Vnm - WMM.S(n - 1, m - 1, dyear) * Wnm);
        py += 0.5 * (-WMM.C(n - 1, m - 1, dyear) * Wnm + WMM.S(n - 1, m - 1, dyear) * Vnm);
      }
      if (m == 1 && n >= 2) {
        px += -WMM.C(n - 1, 0, dyear) * Vnm;
        py += -WMM.C(n - 1, 0, dyear) * Wnm;
      }
      if (n >= 2 && n > m) {
        pz += (n - m) * (-WMM.C(n - 1, m, dyear) * Vnm - WMM.S(n - 1, m, dyear) * Wnm);
      }
    }
  }
  return { -px * 1e-5, -py * 1e-5, -pz * 1e-5 };
}

Elements elementsFromGeodetic(double lat, double lon, double h, double dyear, const ConstModel& WMM)
{
  if (dyear <= WMM.epoch) {
    PRINT_ERROR("The year should be greater than the epoch of the magnetic field model.");
  }

  // 5年ごとに新しいデータが出るので，それを2回分過ぎたら警告する．
  // World Magnetic Model: https://www.ncei.noaa.gov/products/world-magnetic-model
  // FIXME: PRINT_WARN_THROTTLEを実装
  if (dyear - WMM.epoch > 10) {
    PRINT_WARN("It is time to replace the WMM data with the latest version.");
  }

  const auto ecef = geomag::ecefFromGeodetic(lat, lon, h);            // [m]
  const auto mag_field = geomag::magFieldFromECEF(dyear, ecef, WMM);  // [G]
  return geomag::elementsFromMagField(mag_field, lat, lon);
}
}  // namespace geomag
}  // namespace tobas
