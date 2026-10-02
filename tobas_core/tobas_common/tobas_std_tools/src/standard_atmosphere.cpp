// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_std_tools/standard_atmosphere.hpp"

#include <cassert>
#include <cmath>

#include <tobas_math/core.hpp>

namespace tobas
{
namespace st
{
namespace
{
constexpr double kEarthRadius = 6356766.0;      // Earth radius at 45 degrees north latitude [m].
constexpr double kTropopauseAltitude = 1.1e+4;  // Boundary between the troposphere and stratosphere [m].

constexpr double kGasConstant = 8.31432;             // ICAO standard atmosphere [J/K/mol].
constexpr double kSeaLevelPressure = 101325.0;       // ICAO standard atmosphere [Pa].
constexpr double kGravity = 9.80665;                 // ICAO standard atmosphere [m/s^2].
constexpr double kSeaLevelTemperature = 288.15;      // ICAO standard atmosphere [K].
constexpr double kTemperatureLapseRate = -0.0065;    // ICAO standard atmosphere [K/m].
constexpr double kSeaLevelAirMolarMass = 0.0289664;  // ICAO standard atmosphere [kg/mol].
}  // namespace

double gphToAltitude(double gph)
{
  return kEarthRadius * gph / (kEarthRadius - gph);
}

double altitudeToGPH(double altitude)
{
  return kEarthRadius * altitude / (kEarthRadius + altitude);
}

double gphToTemperature(double gph)
{
  if (gph <= kTropopauseAltitude) {
    return kSeaLevelTemperature + kTemperatureLapseRate * gph;
  }
  else {
    throw;  // TODO
  }
}

double altitudeToTemperature(double altitude)
{
  const auto gph = altitudeToGPH(altitude);
  return gphToTemperature(gph);
}

double gphToPressure(double gph)
{
  if (gph <= kTropopauseAltitude) {
    double T = gphToTemperature(gph);
    return temperatureToPressure(T);
  }
  else {
    throw;  // TODO
  }
}

double pressureToTemperature(double p)
{
  assert(p > 0.0);

  constexpr auto kExponent = (kTemperatureLapseRate * kGasConstant) / (kGravity * kSeaLevelAirMolarMass);
  return kSeaLevelTemperature * std::pow(kSeaLevelPressure / p, kExponent);
}

double altitudeToPressure(double altitude)
{
  const auto gph = altitudeToGPH(altitude);
  return gphToPressure(gph);
}

double temperatureToPressure(double T)
{
  assert(T > 0.0);

  constexpr auto kExponent = (kGravity * kSeaLevelAirMolarMass) / (kTemperatureLapseRate * kGasConstant);
  return kSeaLevelPressure * std::pow(kSeaLevelTemperature / T, kExponent);
}

double gphToDensity(double gph)
{
  const auto p = gphToPressure(gph);
  return pressureToDensity(p);
}

double altitudeToDensity(double altitude)
{
  const auto gph = altitudeToGPH(altitude);
  return gphToDensity(gph);
}

double pressureToDensity(double p)
{
  constexpr auto kCoefficient = kSeaLevelAirMolarMass / kGasConstant;
  const auto T = pressureToTemperature(p);
  return kCoefficient * p / T;
}

double pressureToAltitude(double pressure)
{
  assert(pressure > 0.0);

  constexpr auto kTemperatureToAltitudeScale = kSeaLevelTemperature / kTemperatureLapseRate;
  constexpr auto kPressureExponent = -(kTemperatureLapseRate * kGasConstant) / (kGravity * kSeaLevelAirMolarMass);

  const auto gph = kTemperatureToAltitudeScale * (std::pow(pressure / kSeaLevelPressure, kPressureExponent) - 1.0);
  assert(gph < kTropopauseAltitude);  // Error is large above 11 km altitude.

  return gphToAltitude(gph);
}

void pressureToAltitude(double pressure, double pressure_var, double& altitude, double& altitude_var)
{
  constexpr auto kTemperatureToAltitudeScale = kSeaLevelTemperature / kTemperatureLapseRate;
  constexpr auto kPressureExponent = -(kTemperatureLapseRate * kGasConstant) / (kGravity * kSeaLevelAirMolarMass);
  constexpr auto kPressureToAltitudeScale = -kTemperatureToAltitudeScale * kPressureExponent;

  altitude = pressureToAltitude(pressure);

  // Altitude noise / pressure noise ~ 1e-2
  const auto amp = (kPressureToAltitudeScale / pressure) * std::pow(pressure / kSeaLevelPressure, kPressureExponent);
  altitude_var = math::sqr(amp) * pressure_var;
}
}  // namespace st
}  // namespace tobas
