// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

#include <tobas_constants/fixed_wing.hpp>
#include <tobas_math/linalg.hpp>

namespace tobas
{
/**
 * @brief Compute the angle of attack (`alpha`).
 *
 * @param u,w Aircraft velocity relative to wind in the FRD coordinate system [m/s].
 * @return double Angle of attack [rad].
 */
inline double angleOfAttack(const double& u, const double& w)
{
  return u > kMinAirSpeedThresh ? std::atan(w / u) : 0;
}

/**
 * @brief Compute the angle of attack (`alpha`).
 *
 * @param linvel_B Aircraft velocity relative to wind in the FRD coordinate system [m/s].
 * @return double Angle of attack [rad].
 */
inline double angleOfAttack(const Eigen::Vector3d& linvel_B)
{
  return angleOfAttack(linvel_B.x(), linvel_B.z());
}

/**
 * @brief Compute the sideslip angle (`beta`).
 *
 * @param u,v,w Aircraft velocity relative to wind in the FRD coordinate system [m/s].
 * @return double Sideslip angle [rad].
 */
inline double angleOfSideSlip(const double& u, const double& v, const double& w)
{
  const auto V = math::norm(u, v, w);
  return V > kMinAirSpeedThresh ? std::asin(v / V) : 0;
}

/**
 * @brief Compute the sideslip angle (`beta`).
 *
 * @param linvel_B Aircraft velocity relative to wind in the FRD coordinate system [m/s].
 * @return double Sideslip angle [rad].
 */
inline double angleOfSideSlip(const Eigen::Vector3d& linvel_B)
{
  return angleOfSideSlip(linvel_B.x(), linvel_B.y(), linvel_B.z());
}

/**
 * @brief Compute dynamic pressure (`q_bar`).
 *
 * @param rho Air density [kg/m^3].
 * @param V Magnitude of aircraft velocity relative to wind [m/s].
 * @return double Dynamic pressure [Pa].
 */
inline double dynamicPressure(const double& rho, const double& V)
{
  assert(rho > 0);
  assert(V >= 0);
  return rho * math::sqr(V) / 2;
}
}  // namespace tobas
