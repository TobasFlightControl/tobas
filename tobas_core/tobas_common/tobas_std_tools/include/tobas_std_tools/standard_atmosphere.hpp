// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

// Standard atmosphere.
// Ref: https://pigeon-poppo.com/standard-atmosphere/
namespace tobas
{
namespace st
{
/**
 * Calculate geometric altitude from geopotential height.
 *
 * @param gph Geopotential height [m].
 * @return double Geometric altitude [m].
 */
double gphToAltitude(double gph);

/**
 * Calculate geopotential height from geometric altitude.
 *
 * @param altitude Geometric altitude [m].
 * @return double Geopotential height [m].
 */
double altitudeToGPH(double altitude);

/**
 * Calculate standard atmosphere temperature from geopotential height.
 *
 * @param gph Geopotential height [m].
 * @return double Standard atmosphere temperature [K].
 */
double gphToTemperature(double gph);

/**
 * Calculate standard atmosphere temperature from geometric altitude.
 *
 * @param altitude Geometric altitude [m].
 * @return double Standard atmosphere temperature [K].
 */
double altitudeToTemperature(double altitude);

/**
 * Calculate standard atmosphere temperature from atmospheric pressure.
 *
 * @param p Atmospheric pressure [Pa].
 * @return double Standard atmosphere temperature [K].
 *
 * @note Assumes the troposphere.
 */
double pressureToTemperature(double p);

/**
 * Calculate standard atmosphere pressure from geopotential height.
 *
 * @param gph Geopotential height [m].
 * @return double Standard atmosphere pressure [Pa].
 */
double gphToPressure(double gph);

/**
 * Calculate standard atmosphere pressure from geometric altitude.
 *
 * @param altitude Geometric altitude [m].
 * @return double Standard atmosphere pressure [Pa].
 */
double altitudeToPressure(double altitude);

/**
 * Calculate standard atmosphere pressure from atmospheric temperature.
 *
 * @param T Atmospheric temperature [K].
 * @return double Standard atmosphere pressure [Pa].
 *
 * @note Assumes the troposphere.
 */
double temperatureToPressure(double T);

/**
 * Calculate standard atmosphere density from geopotential height.
 *
 * @param gph Geopotential height [m].
 * @return double Standard atmosphere density [kg/m^3].
 */
double gphToDensity(double gph);

/**
 * Calculate standard atmosphere density from geometric altitude.
 *
 * @param altitude Geometric altitude [m].
 * @return double Standard atmosphere density [kg/m^3].
 */
double altitudeToDensity(double altitude);

/**
 * Calculate standard atmosphere density from atmospheric pressure.
 *
 * @param p Atmospheric pressure [Pa].
 * @return double Standard atmosphere density [kg/m^3].
 */
double pressureToDensity(double p);

/**
 * Calculate geometric altitude [m] from atmospheric pressure [Pa], assuming the standard troposphere.
 *
 * @param pressure Atmospheric pressure [Pa].
 * @return double Geometric altitude [m].
 */
double pressureToAltitude(double pressure);

/**
 * Calculate geometric altitude [m] from atmospheric pressure [Pa], assuming the standard troposphere.
 * Also converts the variance.
 *
 * @param pressure Atmospheric pressure [Pa].
 * @param pressure_var Atmospheric pressure variance [Pa^2].
 * @param altitude Geometric altitude [m] (output).
 * @param altitude_var Geometric altitude variance [m^2] (output).
 */
void pressureToAltitude(double pressure, double pressure_var, double& altitude, double& altitude_var);
}  // namespace st
}  // namespace tobas
