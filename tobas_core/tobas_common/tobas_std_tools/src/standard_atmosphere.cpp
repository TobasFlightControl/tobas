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

double gphToAltitude(double _gph)
{
  return kEarthRadius * _gph / (kEarthRadius - _gph);
}

double altitudeToGPH(double _altitude)
{
  return kEarthRadius * _altitude / (kEarthRadius + _altitude);
}

double gphToTemperature(double _gph)
{
  if (_gph <= kTropopauseAltitude) {
    return kSeaLevelTemperature + kTemperatureLapseRate * _gph;
  }
  else {
    throw;  // TODO
  }
}

double altitudeToTemperature(double _altitude)
{
  const auto gph = altitudeToGPH(_altitude);
  return gphToTemperature(gph);
}

double gphToPressure(double _gph)
{
  if (_gph <= kTropopauseAltitude) {
    double T = gphToTemperature(_gph);
    return temperatureToPressure(T);
  }
  else {
    throw;  // TODO
  }
}

double pressureToTemperature(double _p)
{
  assert(_p > 0.0);

  constexpr auto kExponent = (kTemperatureLapseRate * kGasConstant) / (kGravity * kSeaLevelAirMolarMass);
  return kSeaLevelTemperature * std::pow(kSeaLevelPressure / _p, kExponent);
}

double altitudeToPressure(double _altitude)
{
  const auto gph = altitudeToGPH(_altitude);
  return gphToPressure(gph);
}

double temperatureToPressure(double _t)
{
  assert(_t > 0.0);

  constexpr auto kExponent = (kGravity * kSeaLevelAirMolarMass) / (kTemperatureLapseRate * kGasConstant);
  return kSeaLevelPressure * std::pow(kSeaLevelTemperature / _t, kExponent);
}

double gphToDensity(double _gph)
{
  const auto p = gphToPressure(_gph);
  return pressureToDensity(p);
}

double altitudeToDensity(double _altitude)
{
  const auto gph = altitudeToGPH(_altitude);
  return gphToDensity(gph);
}

double pressureToDensity(double _p)
{
  constexpr auto kCoefficient = kSeaLevelAirMolarMass / kGasConstant;
  const auto T = pressureToTemperature(_p);
  return kCoefficient * _p / T;
}

double pressureToAltitude(double _pressure)
{
  assert(_pressure > 0.0);

  constexpr auto kTemperatureToAltitudeScale = kSeaLevelTemperature / kTemperatureLapseRate;
  constexpr auto kPressureExponent = -(kTemperatureLapseRate * kGasConstant) / (kGravity * kSeaLevelAirMolarMass);

  const auto gph = kTemperatureToAltitudeScale * (std::pow(_pressure / kSeaLevelPressure, kPressureExponent) - 1.0);
  assert(gph < kTropopauseAltitude);  // Error is large above 11 km altitude.

  return gphToAltitude(gph);
}

void pressureToAltitude(double _pressure, double _pressure_var, double& _altitude, double& _altitude_var)
{
  constexpr auto kTemperatureToAltitudeScale = kSeaLevelTemperature / kTemperatureLapseRate;
  constexpr auto kPressureExponent = -(kTemperatureLapseRate * kGasConstant) / (kGravity * kSeaLevelAirMolarMass);
  constexpr auto kPressureToAltitudeScale = -kTemperatureToAltitudeScale * kPressureExponent;

  _altitude = pressureToAltitude(_pressure);

  // Altitude noise / pressure noise ~ 1e-2
  const auto amp = (kPressureToAltitudeScale / _pressure) * std::pow(_pressure / kSeaLevelPressure, kPressureExponent);
  _altitude_var = math::sqr(amp) * _pressure_var;
}
}  // namespace st
}  // namespace tobas
