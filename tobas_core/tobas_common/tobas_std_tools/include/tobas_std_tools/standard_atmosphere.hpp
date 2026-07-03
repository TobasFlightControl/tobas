// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

/**
 * @brief Standard atmosphere.
 * cf. https://pigeon-poppo.com/standard-atmosphere/
 */
namespace tobas
{
namespace st
{
/**
 * @brief Calculate geometric altitude from geopotential height.
 *
 * @param gph Geopotential height [m].
 * @return double Geometric altitude [m].
 */
double gphToAltitude(const double& gph);

/**
 * @brief Calculate geopotential height from geometric altitude.
 *
 * @param altitude Geometric altitude [m].
 * @return double Geopotential height [m].
 */
double altitudeToGPH(const double& altitude);

/**
 * @brief Calculate standard atmosphere temperature from geopotential height.
 *
 * @param gph Geopotential height [m].
 * @return double Standard atmosphere temperature [K].
 */
double gphToTemperature(const double& gph);

/**
 * @brief Calculate standard atmosphere temperature from geometric altitude.
 *
 * @param altitude Geometric altitude [m].
 * @return double Standard atmosphere temperature [K].
 */
double altitudeToTemperature(const double& altitude);

/**
 * @brief Calculate standard atmosphere temperature from atmospheric pressure.
 *
 * @param p Atmospheric pressure [Pa].
 * @return double Standard atmosphere temperature [K].
 *
 * @note Assumes the troposphere.
 */
double pressureToTemperature(const double& p);

/**
 * @brief Calculate standard atmosphere pressure from geopotential height.
 *
 * @param gph Geopotential height [m].
 * @return double Standard atmosphere pressure [Pa].
 */
double gphToPressure(const double& gph);

/**
 * @brief Calculate standard atmosphere pressure from geometric altitude.
 *
 * @param altitude Geometric altitude [m].
 * @return double Standard atmosphere pressure [Pa].
 */
double altitudeToPressure(const double& altitude);

/**
 * @brief Calculate standard atmosphere pressure from atmospheric temperature.
 *
 * @param T Atmospheric temperature [K].
 * @return double Standard atmosphere pressure [Pa].
 *
 * @note Assumes the troposphere.
 */
double temperatureToPressure(const double& T);

/**
 * @brief Calculate standard atmosphere density from geopotential height.
 *
 * @param gph Geopotential height [m].
 * @return double Standard atmosphere density [kg/m^3].
 */
double gphToDensity(const double& gph);

/**
 * @brief Calculate standard atmosphere density from geometric altitude.
 *
 * @param altitude Geometric altitude [m].
 * @return double Standard atmosphere density [kg/m^3].
 */
double altitudeToDensity(const double& altitude);

/**
 * @brief Calculate standard atmosphere density from atmospheric pressure.
 *
 * @param p Atmospheric pressure [Pa].
 * @return double Standard atmosphere density [kg/m^3].
 */
double pressureToDensity(const double& p);

/**
 * @brief Calculate geometric altitude [m] from atmospheric pressure [Pa], assuming the standard troposphere.
 *
 * @param pressure Atmospheric pressure [Pa].
 * @return double Geometric altitude [m].
 */
double pressureToAltitude(const double& pressure);

/**
 * @brief Calculate geometric altitude [m] from atmospheric pressure [Pa], assuming the standard troposphere.
 * Also converts the variance.
 *
 * @param pressure Atmospheric pressure [Pa].
 * @param pressure_var Atmospheric pressure variance [Pa^2].
 * @param altitude Geometric altitude [m] (output).
 * @param altitude_var Geometric altitude variance [m^2] (output).
 */
void pressureToAltitude(const double& pressure, const double& pressure_var, double& altitude, double& altitude_var);
}  // namespace st
}  // namespace tobas
