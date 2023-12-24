#include <cmath>
#include <cassert>
#include <stdexcept>

#include "../include/dh_std_tools/standard_atmosphere.hpp"
#include "../include/dh_std_tools/math.hpp"

#define R0 6356766.                 // 北緯45度における地球の半径 [m]
#define TROPOPAUSE_ALTITUDE 1.1e+4  // 対流圏界面 (対流圏と成層圏の境界面) [m]

// ICAO標準大気
#define R 8.31432    // 気体定数 [J/K/mol]
#define P0 101325.   // 海面気圧 [Pa]
#define G 9.80665    // 重力加速度 [m/s^2]
#define T0 288.15    // 海面気温 [K]
#define L -0.0065    // 気温減率 [K/m]
#define M 0.0289664  // 海面大気のモル質量 [kg/mol]

using namespace std;

namespace dh_std
{
double gphToAltitude(const double& gph)
{
  return R0 * gph / (R0 - gph);
}

double altitudeToGPH(const double& altitude)
{
  return R0 * altitude / (R0 + altitude);
}

double gphToTemperature(const double& gph)
{
  if (gph <= TROPOPAUSE_ALTITUDE)
  {
    return T0 + L * gph;
  }
  else
  {
    throw;  // TODO
  }
}

double altitudeToTemperature(const double& altitude)
{
  const auto gph = altitudeToGPH(altitude);
  return gphToTemperature(gph);
}

double gphToPressure(const double& gph)
{
  if (gph <= TROPOPAUSE_ALTITUDE)
  {
    double T = gphToTemperature(gph);
    return temperatureToPressure(T);
  }
  else
  {
    throw;  // TODO
  }
}

double pressureToTemperature(const double& p)
{
  assert(p > 0.);

  constexpr auto exp = (L * R) / (G * M);
  return T0 * pow(P0 / p, exp);
}

double altitudeToPressure(const double& altitude)
{
  const auto gph = altitudeToGPH(altitude);
  return gphToPressure(gph);
}

double temperatureToPressure(const double& T)
{
  assert(T > 0.);

  constexpr auto exp = (G * M) / (L * R);
  return P0 * pow(T0 / T, exp);
}

double gphToDensity(const double& gph)
{
  const auto p = gphToPressure(gph);
  return pressureToDensity(p);
}

double altitudeToDensity(const double& altitude)
{
  const auto gph = altitudeToGPH(altitude);
  return gphToDensity(gph);
}

double pressureToDensity(const double& p)
{
  constexpr auto c = M / R;
  const auto T = pressureToTemperature(p);
  return c * p / T;
}

double pressureToAltitude(const double& pressure)
{
  assert(pressure > 0.);

  constexpr auto a = T0 / L;
  constexpr auto b = -(L * R) / (G * M);

  const auto gph = a * (pow(pressure / P0, b) - 1.);
  assert(gph < TROPOPAUSE_ALTITUDE);  // 高度11km以上では誤差が大きい

  return gphToAltitude(gph);
}

void pressureToAltitude(
  const double& pressure,
  const double& pressure_var,
  double& altitude,
  double& altitude_var)
{
  constexpr auto a = T0 / L;
  constexpr auto b = -(L * R) / (G * M);
  constexpr auto c = -a * b;

  altitude = pressureToAltitude(pressure);

  const auto amp = (c / pressure) * pow(pressure / P0, b);  // 高度ノイズ/気圧ノイズ ~ 1e-2
  altitude_var = sqr(amp) * pressure_var;
}
}  // namespace dh_std
