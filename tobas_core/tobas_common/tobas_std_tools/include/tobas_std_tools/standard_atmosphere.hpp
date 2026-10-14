// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

// Standard atmosphere.
// Ref: https://pigeon-poppo.com/standard-atmosphere/
namespace tobas::st
{
/**
 * Calculate geometric altitude from geopotential height.
 *
 * @param _gph Geopotential height [m].
 * @return double Geometric altitude [m].
 */
double gphToAltitude(double _gph);

/**
 * Calculate geopotential height from geometric altitude.
 *
 * @param _altitude Geometric altitude [m].
 * @return double Geopotential height [m].
 */
double altitudeToGPH(double _altitude);

/**
 * Calculate standard atmosphere temperature from geopotential height.
 *
 * @param _gph Geopotential height [m].
 * @return double Standard atmosphere temperature [K].
 */
double gphToTemperature(double _gph);

/**
 * Calculate standard atmosphere temperature from geometric altitude.
 *
 * @param _altitude Geometric altitude [m].
 * @return double Standard atmosphere temperature [K].
 */
double altitudeToTemperature(double _altitude);

/**
 * Calculate standard atmosphere temperature from atmospheric pressure.
 *
 * @param _p Atmospheric pressure [Pa].
 * @return double Standard atmosphere temperature [K].
 *
 * @note Assumes the troposphere.
 */
double pressureToTemperature(double _p);

/**
 * Calculate standard atmosphere pressure from geopotential height.
 *
 * @param _gph Geopotential height [m].
 * @return double Standard atmosphere pressure [Pa].
 */
double gphToPressure(double _gph);

/**
 * Calculate standard atmosphere pressure from geometric altitude.
 *
 * @param _altitude Geometric altitude [m].
 * @return double Standard atmosphere pressure [Pa].
 */
double altitudeToPressure(double _altitude);

/**
 * Calculate standard atmosphere pressure from atmospheric temperature.
 *
 * @param _t Atmospheric temperature [K].
 * @return double Standard atmosphere pressure [Pa].
 *
 * @note Assumes the troposphere.
 */
double temperatureToPressure(double _t);

/**
 * Calculate standard atmosphere density from geopotential height.
 *
 * @param _gph Geopotential height [m].
 * @return double Standard atmosphere density [kg/m^3].
 */
double gphToDensity(double _gph);

/**
 * Calculate standard atmosphere density from geometric altitude.
 *
 * @param _altitude Geometric altitude [m].
 * @return double Standard atmosphere density [kg/m^3].
 */
double altitudeToDensity(double _altitude);

/**
 * Calculate standard atmosphere density from atmospheric pressure.
 *
 * @param _p Atmospheric pressure [Pa].
 * @return double Standard atmosphere density [kg/m^3].
 */
double pressureToDensity(double _p);

/**
 * Calculate geometric altitude [m] from atmospheric pressure [Pa], assuming the standard troposphere.
 *
 * @param _pressure Atmospheric pressure [Pa].
 * @return double Geometric altitude [m].
 */
double pressureToAltitude(double _pressure);

/**
 * Calculate geometric altitude [m] from atmospheric pressure [Pa], assuming the standard troposphere.
 * Also converts the variance.
 *
 * @param _pressure Atmospheric pressure [Pa].
 * @param _pressure_var Atmospheric pressure variance [Pa^2].
 * @param _altitude Geometric altitude [m] (output).
 * @param _altitude_var Geometric altitude variance [m^2] (output).
 */
void pressureToAltitude(double _pressure, double _pressure_var, double& _altitude, double& _altitude_var);
}  // namespace tobas::st
